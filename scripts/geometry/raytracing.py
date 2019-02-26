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

        p0l0 = self.planePoint - ray.rayPoint
        t = self.planeNormal.dot(p0l0) / denom

        if t >= 0:
            return t * ray.rayDirection + ray.rayPoint
        else:
            return False

class Ray:

    def __init__(self, rayPoint, ray2point, energy=0):

        self.rayPoint = rayPoint

        self.rayDirection = ray2point - rayPoint
        self.rayDirection = self.rayDirection/np.linalg.norm(self.rayDirection)

        self.energy = energy
