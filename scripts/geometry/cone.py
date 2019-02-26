import numpy as np
import math as m

import solid_angle
from pyquaternion import Quaternion
from raytracing import Ray
import geometry.solid_angle

class Cone:

    def __init__(self, origin, direction, angle):

        self.origin = origin
        self.direction = direction/np.linalg.norm(direction)
        self.angle = angle

        self.cone_axis_projector = np.dot(np.matrix([direction]).transpose(), np.matrix([direction]))

        self.cone_ray = Ray(self.origin, self.origin + self.direction)

    # #{ projectPoint()
    
    def projectPoint(self, point):
    
        # shift the point by the origin
        # so we can use the cone_axis_projector as an orthogonal projector
        # on a subspace
        point_vec = point - self.origin
        point_axis_angle = solid_angle.vector_angle(point_vec, self.direction)
    
        # project the point to the axis, and shift it back
        axis_projection = np.dot(self.cone_axis_projector, point_vec) + self.origin
    
        axis_rot = np.linalg.cross(self.direction, point_vec)
        axis_rot = axis_rot/np.linalg.norm(axis_rot)
    
        my_quaternion = Quaternion(axis=axis_rot, angle=self.angle - point_axis_angle)
    
        point_on_cone = my_quaternion.rotate(point_vec) + self.origin
    
        vec_point_on_cone = point_on_cone - self.origin
        vec_point_on_cone = vec_point_on_cone/np.linalg.norm(vec_point_on_cone)
    
        beta = self.angle - point_axis_angle
    
        if point_axis_angle < self.angle:
    
            ortho_projection = self.origin + vec_point_on_cone*m.cos(beta)*np.linalg.norm(point_vec)
            color = 'rgb(255, 0, 0)'
    
        elif point_axis_angle >= self.angle and point_axis_angle <= m.pi - self.angle:
    
            ortho_projection = self.origin + vec_point_on_cone*m.cos(point_axis_angle-self.angle)*np.linalg.norm(point_vec)
            color = 'rgb(0, 255, 0)'
        else:
    
            ortho_projection = self.origin
            color = 'rgb(0, 0, 255)'
    
        return np.array([ortho_projection[0], ortho_projection[1], ortho_projection[2]]), color
    
    # #} end of projectPoint()

    # #{ projectPointOnPlane()
    
    def projectPointOnPlane(self, point, plane):
    
        # find the intersection of the plane and the cone's axis
        intersect = plane.intersectionRay(self.cone_ray)
    
        if not isinstance(intersect, np.ndarray):
            return False

        vec_to_intersect = intersect - self.origin
        vec_to_intersect = vec_to_intersect/np.linalg.norm(vec_to_intersect)

        vec_to_point = point - self.origin
        vec_to_point = vec_to_point/np.linalg.norm(vec_to_point)

        my_axis = np.cross(self.direction, vec_to_point)
        my_axis = my_axis/np.linalg.norm(my_axis)

        my_quaternion = Quaternion(axis=my_axis, angle=self.angle)

        vec_along_cone = my_quaternion.rotate(vec_to_intersect)
        ray_along_cone = Ray(self.origin, self.origin + vec_along_cone)

        intersect2 = plane.intersectionRay(ray_along_cone)

        return intersect2
    
    # #} end of projectPointOnPlane()
