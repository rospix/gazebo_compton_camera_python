import numpy as np
from geometry.raytracing import Plane
import shapely as sh
from shapely.geometry import Polygon
from shapely import speedups
speedups.enable()
import copy

class Polygon3D:

    # #{ __init__()

    def __init__(self, points):

        self.points_3d = points

        if len(points) < 3:

            print("Polygon3D: cannot create a polygon from < 3 vertices!")

            return False

        # basis of the linear subspace (the plane moved to the origin)
        self.basis = []

        # projector to the linear subspace (the plane moved to the origin)
        self.projector = []

        # projection of the origin
        self.zero_point = []

        # all the 2d points on the polygon
        self.points_2d = []

        # sympy polygon
        self.polygon_2d = []

        # find the normal vector of the plane
        v1 = points[1] - points[0]
        v2 = points[2] - points[0]
        self.normal_vector = np.cross(v1, v2)
        self.normal_vector = self.normal_vector / np.linalg.norm(self.normal_vector)

        # the defining point of the plane
        self.zero_point = points[0]

        self.plane = Plane(self.zero_point, self.normal_vector)

        # # find the orthogonal basis of the plane

        # first vector of the orthonormal basis
        if np.abs(self.normal_vector[0]) > 1e-3 or np.abs(self.normal_vector[1]) > 1e-3:
            basis1 = np.array([-self.normal_vector[1], self.normal_vector[0], self.normal_vector[2]])
        else:
            basis1 = np.array([self.normal_vector[0], -self.normal_vector[2], self.normal_vector[1]])

        # second vector of the orthonormal basis
        basis2 = np.cross(self.normal_vector, basis1)
        basis2 = basis2 / np.linalg.norm(basis2)

        self.basis = np.matrix([basis1, basis2, self.normal_vector]).transpose()

        # projector to orthogonal adjucent of the planes normal
        self.projector = self.basis*self.basis.transpose()

        for i,point in enumerate(points):
            self.points_2d.append(self.projectOn2D(point))

        self.polygon_2d = Polygon(self.points_2d)

    # #}

    # #{ projectOn2D()

    def projectOn2D(self, point_in):

        point = np.matrix([point_in - self.zero_point]).transpose()

        # project the point on the affine subspace of the plane
        projection = self.projector*point

        # express in the orthonormal basis of the subspace
        projection = np.dot(np.linalg.inv(self.basis), projection)

        return np.array([projection[0, 0], projection[1, 0]])

    # #} end of

    # #{ intersection()

    def intersection(self, intersector):

        intersectee_3d = self.plane.intersectionRay(intersector)

        if not isinstance(intersectee_3d, np.ndarray):
            return False

        intersectee_copy = copy.deepcopy(intersectee_3d)

        intersectee_2d = sh.geometry.Point(self.projectOn2D(intersectee_copy))

        if self.polygon_2d.contains(intersectee_2d):
            return intersectee_3d
        else:
            return False

    # #} end of rayCollision()

    # #{ plotVertices()
    
    def plotVertices(self):
    
        xs = [point[0] for point in self.points_3d]
        ys = [point[1] for point in self.points_3d]
        zs = [point[2] for point in self.points_3d]
    
        return xs, ys, zs
    
    # #} end of plotVertices()
