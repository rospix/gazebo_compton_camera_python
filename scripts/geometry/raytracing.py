import numpy as np
import math as m
import sympy as sp

class Plane:

    def __init__(self, planePoint, planeNormal):

        self.planePoint = planePoint

        self.planeNormal = planeNormal/np.linalg.norm(planeNormal)

    def intersectionRay(self, ray, epsilon=1e-6):

        denom = self.planeNormal.dot(ray.rayDirection)
        if abs(denom) < epsilon:
            return False

        t = self.planeNormal.dot(self.planePoint - ray.rayPoint) / denom

        if t >= 0:
            return True, t * ray.rayDirection + ray.rayPoint
        else:
            return False, 0

class Ray:

    def __init__(self, rayPoint, ray2point, energy=0):

        self.rayPoint = rayPoint

        self.rayDirection = ray2point - rayPoint
        self.rayDirection = self.rayDirection/np.linalg.norm(self.rayDirection)

        self.energy = energy
