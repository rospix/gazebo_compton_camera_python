import numpy as np
from geometry.raytracing import Plane

class Rectangle3D:

    # #{ __init__()

    def __init__(self, points):

        self.points_3d = points

        if len(points) < 3:

            print("Polygon3D: cannot create a polygon from < 3 vertices!")

            return False

        # basis of the linear subspace (the plane moved to the origin)
        self.ortho_basis = []

        self.rectangle_basis = []

        # projector to the linear subspace (the plane moved to the origin)
        self.projector = []

        # projection of the origin
        self.zero_point = []

        # find the normal vector of the plane
        self.v1 = points[1] - points[0]
        self.v2 = points[3] - points[0]
        self.normal_vector = np.cross(self.v1, self.v2)
        self.normal_vector = self.normal_vector / np.linalg.norm(self.normal_vector)

        # the defining point of the plane
        self.zero_point = points[0]

        self.plane = Plane(self.zero_point, self.normal_vector)

        ortho_basis1 = self.v1/np.linalg.norm(self.v1)
        ortho_basis2 = self.v2/np.linalg.norm(self.v2)

        self.ortho_basis = np.matrix([ortho_basis1, ortho_basis2, self.normal_vector]).transpose()
        self.ortho_basis_inv = np.linalg.inv(self.ortho_basis)

        self.rectangle_basis = np.matrix([self.v1, self.v2, self.normal_vector]).transpose()
        self.rectangle_basis_inv = np.linalg.inv(self.rectangle_basis)

        # projector to orthogonal adjucent of the planes normal
        self.projector = np.dot(self.ortho_basis, self.ortho_basis.T)

    # #}

    # #{ intersection()

    def intersection(self, intersector):

        success, intersectee_3d = self.plane.intersectionRay(intersector)

        if not success:
            return False

        # project the point on the affine subspace of the plane
        projection = np.dot(self.projector, (intersectee_3d - self.zero_point).T)

        # express in the orthonormal basis of the subspace
        projection = np.dot(self.rectangle_basis_inv, projection.T)

        if projection[0, 0] >= 0.0 and projection[0, 0] <= 1.0 and projection[1, 0] >= 0.0 and projection[1, 0] <= 1.0:
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
