import numpy as np

class ConvexPolygon:

    def __init__(self, points):

        self.n = len(points)+1

        self.vert = points
        self.vert.append(points[-1])

        self.total_area = self.area()

    def triangleArea(self, a, b, c):

        return np.abs((a[0] * b[1] + b[0] * c[1] + c[0] * a[1] - a[1] * b[0] - b[1] * c[0] - c[1] * a[0]) / 2.0);

    def isPointIn(self, point):

        area = 0

        for i in range(0, self.n):
            area += self.triangleArea(self.vert[i], self.vert[i-1], point);

        if area > (self.total_area * 1.001):
            return False
        else:
            return True

    def area(self):

        cot = np.array([0, 0])

        for i in range(0, self.n-1):
            cot[0] += self.vert[i][0]
            cot[1] += self.vert[i][1]

        cot[0] /= len(self.vert)
        cot[1] /= len(self.vert)

        area = 0;

        for i in range(1, self.n):
            area += self.triangleArea(self.vert[i], self.vert[i - 1], cot)

        return area

