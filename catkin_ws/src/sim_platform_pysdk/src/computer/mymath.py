import math

class Math(object):
    @staticmethod
    def points2vector(pt1, pt2):
        return [pt2[0] - pt1[0], pt2[1] - pt1[1]]

    @staticmethod
    def cross(vector1, vector2):
        return vector1[0] * vector2[1] - vector1[1] * vector2[0]

    @staticmethod
    def dot(vector1, vector2):
        return vector1[0] * vector2[0] + vector1[1] * vector2[1]

    @staticmethod
    def linesintersect(lineA, lineB):
        A1, A2 = lineA
        B1, B2 = lineB
        A1B1 = Math.points2vector(A1, B1)
        A1A2 = Math.points2vector(A1, A2)
        A1B2 = Math.points2vector(A1, B2)
        
        B1A1 = Math.points2vector(B1, A1)
        B1B2 = Math.points2vector(B1, B2)
        B1A2 = Math.points2vector(B1, A2)

        if Math.cross(B1B2, B1A1) * Math.cross(B1B2, B1A2) <= 0 and Math.cross(A1A2, A1B1) *  Math.cross(A1A2, A1B2) <= 0:
            return True
        else:
            return False

    @staticmethod
    def norm(vector):
        return math.sqrt(vector[0]**2 + vector[1]**2)

    @staticmethod
    def is_nul_vector(vector):
        for i in range(2):
            if vector[i] != 0:
                return False
        return True

    @staticmethod
    def vectorsangle(vector1, vector2):
        cos = Math.dot(vector1, vector2) / (Math.norm(vector1) * Math.norm(vector2))
        if math.fabs(cos - 1.0) < 0.0000001:
            arccos = 0
        elif math.fabs(cos - (-1.0)) < 0.0000001:
            arccos = math.pi
        else:
            arccos = math.acos(cos)
        if Math.cross(vector1, vector2) >= 0:
            return arccos
        else:
            return math.pi + arccos

    @staticmethod
    def gen_convex_hull(points):
        # Initialize
        min_y = points[0][1]
        min_y_pts = [points[0]]
        for point in points:
            if point[1] == min_y:
                min_y_pts.append(point)
            elif point[1] < min_y:
                min_y = point[1]
                min_y_pts = [point]
        min_x_pt = min_y_pts[0]
        for point in min_y_pts:
            if point[0] < min_x_pt[0]:
                min_x_pt = point
        start_pt = min_x_pt
        
        # Search
        convex_hull = [start_pt]
        while True:
            list_tmp = []
            for point in points:
                if point in convex_hull[1:] or point == convex_hull[-1]:
                    continue
                if len(convex_hull) == 1:
                    vector1 = [1, 0] 
                else:
                    vector1 = Math.points2vector(convex_hull[-2], convex_hull[-1])
                vector2 = Math.points2vector(convex_hull[-1], point)
                angle = Math.vectorsangle(vector1, vector2)
                list_tmp.append((point, angle, ))
            list_tmp = sorted(list_tmp, key=lambda term: term[1])
            if not Math.is_nul_vector(Math.points2vector(list_tmp[0][0], convex_hull[0])):
                convex_hull.append(list_tmp[0][0])
                while list_tmp[0][0] in points:
                    points.remove(list_tmp[0][0])
            else:
                break
        return convex_hull

    @staticmethod
    def expand_rectangles(rect_list, r):
        list_tmp = []
        for rect in rect_list:
            tmp = [
                rect[0] - r,
                rect[1] + r,
                rect[2] - r,
                rect[3] + r
            ]
            list_tmp.append(tmp)
        return list_tmp

    @staticmethod
    def points_dist(point1, point2):
        vector = Math.points2vector(point1, point2)
        return math.sqrt(vector[0]**2 + vector[1]**2)

    @staticmethod
    def choose_points_at_right_hand(start, end, points):
        right_points = []
        boundary_vector = Math.points2vector(start, end)
        for point in points:
            vector = Math.points2vector(start, point)
            if Math.cross(vector, boundary_vector) >= 0 and point != start:
                right_points.append((point, Math.points_dist(point, start), ))
        right_points = sorted(right_points, key=lambda term: term[1])
        right_points = [term[0] for term in right_points]
        # import matplotlib.pyplot as plt
        # plt.scatter([x[0] for x in right_points], [x[1] for x in right_points], color='yellow', s=100, alpha=0.5)
        # plt.show()
        return right_points

    @staticmethod
    def choose_points_at_left_hand(start, end, points):
        left_points = []
        boundary_vector = Math.points2vector(start, end)
        for point in points:
            vector = Math.points2vector(start, point)
            if Math.cross(vector, boundary_vector) <= 0 and point != start:
                left_points.append((point, Math.points_dist(point, start), ))
        left_points = sorted(left_points, key=lambda term: term[1])
        left_points = [term[0] for term in left_points]
        # import matplotlib.pyplot as plt
        # plt.scatter([x[0] for x in left_points], [x[1] for x in left_points], color='yellow', s=100, alpha=0.5)
        # plt.show()
        return left_points