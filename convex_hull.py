from which_pyqt import PYQT_VER
if PYQT_VER == 'PYQT5':
    from PyQt5.QtCore import QLineF, QPointF, QObject
else:
    raise Exception('Unsupported Version of PyQt: {}'.format(PYQT_VER))


import time

RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

PAUSE = 0.25

# This class is used to hold a point object,
# it holds the QpointF value, the points own index,
# and the indexs of its next and previous points in clockwise order.
# The space complexity associated with these objects is constant.
class Point:
    def __init__(self, q_point: QPointF, self_index, prev_index, next_index):
        self.qPoint = q_point
        self.prev = prev_index
        self.next = next_index
        self.index = self_index

    def get_point(self):
        return self.qPoint

    def x(self):
        return self.qPoint.x()

    def y(self):
        return self.qPoint.y()

# This is a hull class that holds an array of point objects and a dictionary for fast lookup times by index.
# The space complexity associated with this object in the algorithm as a whole is O(n)
class Hull:
    def __init__(self, points, length):
        self.points = points
        self.length = length
        self.nodeList = []
        self.node_dict = {}

    # The time and space complexity here is constant.
    def append(self, q_point, index, prev_index, next_index):
        new_point = Point(q_point, index, prev_index, next_index)
        self.node_dict[index] = new_point
        self.nodeList.append(new_point)

    # The time and space for this function is O(n) because each node is touched to create a hull.
    def create(self, points):
        for i in range(len(points)):
            if i == 0:
                if len(points) == 1:
                    self.append(points[i], 0, 0, 0)
                else:
                    self.append(points[i], 0, len(points) - 1, 1)
            elif i == (len(points) - 1):
                self.append(points[i], len(points) - 1, len(points) - 2, 0)
            else:
                self.append(points[i], i, i - 1, i + 1)

    # The following three get functions all run in constant time and space.
    def get_node(self, index):
        if index in self.node_dict:
            return self.node_dict[index]
        return None

    def get_next(self, index):
        if index in self.node_dict:
            return self.node_dict[self.node_dict[index].next]
        return None

    def get_prev(self, index):
        if index in self.node_dict:
            return self.node_dict[self.node_dict[index].prev]
        return None

    # These two get functions run in O(n) time, because they iterate through each node in the hull to find the
    # left and right most nodes. It is also constant space because constant amounts of new memory is allocated.
    def get_left_most(self):
        min_x_value = self.get_node(0).x()
        leftmost_node = self.get_node(0)
        for i in range(self.length):
            if self.get_node(i).x() < min_x_value:
                min_x_value = self.get_node(i).x()
                leftmost_node = self.get_node(i)
        return leftmost_node

    def get_right_most(self):
        max_x_value = self.get_node(0).x()
        rightmost_node = self.get_node(0)
        for i in range(self.length):
            if self.get_node(i).x() > max_x_value:
                max_x_value = self.get_node(i).x()
                rightmost_node = self.get_node(i)
        return rightmost_node

    # Merge runs in O(n) time. Each node is touched in the left and right hulls. It also takes up O(n) space because a
    # new hull is created to be returned from merge.
    def hull_merge(self, self_start, self_end, new_start, new_end, right_hull):
        merged_array = []
        current_node = self_start
        while current_node.get_point() != self_end.get_point():
            merged_array.append(current_node.get_point())
            current_node = self.get_next(current_node.index)
        merged_array.append(self_end.get_point())
        new_current_node = new_start
        while new_current_node.get_point() != new_end.get_point():
            merged_array.append(new_current_node.get_point())
            new_current_node = right_hull.get_next(new_current_node.index)
        merged_array.append(new_end.get_point())
        new_hull = Hull(merged_array, len(merged_array))
        new_hull.create(merged_array)
        return new_hull


class ConvexHullSolver(QObject):

    def __init__(self):
        super().__init__()
        self.pause = False

    def showTangent(self, line, color):
        self.view.addLines(line,color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseTangent(self, line):
        self.view.clearLines(line)

    def blinkTangent(self, line, color):
        self.showTangent(line, color)
        self.eraseTangent(line)

    def showHull(self, polygon, color):
        self.view.addLines(polygon, color)
        if self.pause:
            time.sleep(PAUSE)

    def eraseHull(self, polygon):
        self.view.clearLines(polygon)

    def showText(self, text):
        self.view.displayStatusText(text)

    # Calculates the slope of two QpointF objects. Runs in constant time and space.
    def get_slope(self, point1, point2):
        if point1.x() == point2.x():
            return float('inf')
        return (point2.y() - point1.y()) / (point2.x() - point1.x())

    # The following two functions calculate the upper and lower tangent points on the left and right hulls.
    # Each runs in at worst linear time and constant space, but average case will be much less than linear because
    # of the while loops rather than iterating through every point in both hulls.
    def calc_upper(self, leftmost, rightmost, left, right):
        p = rightmost
        q = leftmost
        temp = self.get_slope(p.get_point(), q.get_point())
        done = False
        while not done:
            done = True
            while temp > self.get_slope(left.get_node(p.prev).get_point(), q.get_point()):
                temp = self.get_slope(left.get_node(p.prev).get_point(), q.get_point())
                p = left.get_node(p.prev)
                done = False
            while temp < self.get_slope(p.get_point(), right.get_node(q.next).get_point()):
                temp = self.get_slope(p.get_point(), right.get_node(q.next).get_point())
                q = right.get_node(q.next)
                done = False
        return p, q

    def calc_lower(self, leftmost, rightmost, left, right):
        p = rightmost
        q = leftmost
        temp = self.get_slope(p.get_point(), q.get_point())
        done = False
        while not done:
            done = True
            while temp < self.get_slope(left.get_node(p.next).get_point(), q.get_point()):
                temp = self.get_slope(left.get_node(p.next).get_point(), q.get_point())
                p = left.get_node(p.next)
                done = False
            while temp > self.get_slope(p.get_point(), right.get_node(q.prev).get_point()):
                temp = self.get_slope(p.get_point(), right.get_node(q.prev).get_point())
                q = right.get_node(q.prev)
                done = False
        return p, q

    # This function calls the functions that calculate the upper and lower tangents. It runs in constant time and space.
    def calc_tangents(self, leftmost, rightmost, left, right):
        if left.length == 2 and right.length == 1:
            if left.get_node(0).y() > left.get_node(1).y():
                l_up_tang = left.get_node(0)
                l_low_tang = left.get_node(1)
            else:
                l_up_tang = left.get_node(1)
                l_low_tang = left.get_node(0)
            r_up_tang = right.get_node(0)
            r_low_tang = right.get_node(0)
        elif right.length == 2 and left.length == 1:
            if right.get_node(0).y() > right.get_node(1).y():
                r_up_tang = right.get_node(0)
                r_low_tang = right.get_node(1)
            else:
                r_up_tang = right.get_node(1)
                r_low_tang = right.get_node(0)
            l_up_tang = left.get_node(0)
            l_low_tang = left.get_node(0)
        else:
            l_up_tang, r_up_tang = self.calc_upper(leftmost, rightmost, left, right)
            l_low_tang, r_low_tang = self.calc_lower(leftmost, rightmost, left, right)
        return l_up_tang, r_up_tang, l_low_tang, r_low_tang

    # This is my function that I use to handle edge cases and to call merge on my two hulls. It runs in constant time
    # and space as well, but the function that it returns runs in linear time and space.
    def merge(self, left_half, right_half):
        rightmost = left_half.get_right_most()
        leftmost = right_half.get_left_most()
        l_up_tang, r_up_tang, l_low_tang, r_low_tang = self.calc_tangents(leftmost, rightmost, left_half, right_half)
        edge_case_merge = []
        if (left_half.length == 2 and right_half.length == 1) or (left_half.length == 1 and right_half.length == 2) or (left_half.length == 2 and right_half.length == 2):
            if l_up_tang.get_point() == l_low_tang.get_point():
                slope_up_tang = self.get_slope(l_up_tang.get_point(), r_up_tang.get_point())
                slope_low_tang = self.get_slope(l_up_tang.get_point(), r_low_tang.get_point())
                if slope_up_tang > slope_low_tang:
                    edge_case_merge.append(r_up_tang.get_point())
                    edge_case_merge.append(r_low_tang.get_point())
                else:
                    edge_case_merge.append(r_low_tang.get_point())
                    edge_case_merge.append(r_up_tang.get_point())
                edge_case_merge.append(l_up_tang.get_point())
            elif r_up_tang.get_point() == r_low_tang.get_point():
                slope_up_tang = self.get_slope(r_up_tang.get_point(), l_up_tang.get_point())
                slope_low_tang = self.get_slope(r_up_tang.get_point(), l_low_tang.get_point())
                if slope_up_tang > slope_low_tang:
                    edge_case_merge.append(l_up_tang.get_point())
                    edge_case_merge.append(l_low_tang.get_point())
                else:
                    edge_case_merge.append(l_low_tang.get_point())
                    edge_case_merge.append(l_up_tang.get_point())
                edge_case_merge.append(r_up_tang.get_point())
            else:
                edge_case_merge.append(l_low_tang.get_point())
                edge_case_merge.append(l_up_tang.get_point())
                edge_case_merge.append(r_up_tang.get_point())
                edge_case_merge.append(r_low_tang.get_point())
            edge_case_hull = Hull(edge_case_merge, len(edge_case_merge))
            edge_case_hull.create(edge_case_merge)
            return edge_case_hull
        else:
            return left_half.hull_merge(l_low_tang, l_up_tang, r_up_tang, r_low_tang, right_half)

    # This is the function that handles the recursive element of my algorith. It runs log(n) times and does n work each
    # call and allocates n amount of space each call.
    def convex_hull(self, points: Hull):
        if points.length < 3:
            return points
        half = points.length // 2
        left_half = Hull(points.points[:half], len(points.points[:half]))
        right_half = Hull(points.points[half:], len(points.points[half:]))
        left_half.create(points.points[:half])
        right_half.create(points.points[half:])
        return self.merge(self.convex_hull(left_half), self.convex_hull(right_half))

    def compute_hull( self, points, pause, view):
        self.pause = pause
        self.view = view
        assert(type(points) == list and type(points[0]) == QPointF)
        t1 = time.time()
        sortedPoints = sorted(points, key=lambda point: point.x())
        t2 = time.time()
        t3 = time.time()
        initial_hull = Hull(sortedPoints, len(sortedPoints))
        initial_hull.create(sortedPoints)
        polygon = self.convex_hull(initial_hull)
        t4 = time.time()
        new_polygon = [QLineF(polygon.points[i], polygon.points[(i + 1) % polygon.length]) for i in range(polygon.length)]
        self.showHull(new_polygon, RED)
        self.showText('Time Elapsed (Convex Hull): {:3.3f} sec'.format(t4-t3))
