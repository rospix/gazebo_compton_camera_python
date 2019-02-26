#!/usr/bin/env python2

import rospy
import tf
import os

import time
import copy

import random
# random.seed(1005)

import math as m
import numpy as np
from pyquaternion import Quaternion

# plotly
import plotly.offline as py
import plotly.graph_objs as go
from scipy.spatial import Delaunay # for triangulation of the cones

# my stuff
import geometry.solid_angle
import photon_attenuation.materials as materials
import photon_attenuation.physics as physics
from geometry.raytracing import Plane, Ray
from geometry.cone import Cone
from geometry.polygon3d import Polygon3D

from nav_msgs.msg import Odometry
from gazebo_rad_msgs.msg import RadiationSource

simulate_energy_noise = True
simulate_pixel_uncertainty = True

# #{ class Source

class Source:

    def __init__(self, energy, activity, position):

        self.energy = energy
        self.activity = activity
        self.position = position
        self.aparent_activity = 0

# #} end of class Source

# #{ class Detector

class Detector:

    def __init__(self, material, thickness, position):

        self.size = 0.01408 # [mm]
        self.vertices = [] # in the frame of the sensor

        self.front = []
        self.back = []
        self.sides = []

        self.material = material
        self.thickness = thickness
        self.position = position

        # FRONT

        # give me the 4 front_vertices of the detector
        a = np.array([self.thickness/2, -self.size/2.0, self.size/2.0]) + position
        b = np.array([self.thickness/2, self.size/2.0, self.size/2.0]) + position
        c = np.array([self.thickness/2, self.size/2.0, -self.size/2.0]) + position
        d = np.array([self.thickness/2, -self.size/2.0, -self.size/2.0]) + position

        # create the sympy front plane
        self.front = Polygon3D([a, b, c, d])

        # BACK

        # give me the 4 back_vertices of the detector
        e = np.array([-self.thickness/2, -self.size/2.0, self.size/2.0]) + position
        f = np.array([-self.thickness/2, self.size/2.0, self.size/2.0]) + position
        g = np.array([-self.thickness/2, self.size/2.0, -self.size/2.0]) + position
        h = np.array([-self.thickness/2, -self.size/2.0, -self.size/2.0]) + position

        # create the sympy back plane
        self.back = Polygon3D([e, f, g, h])

        # orthogonal basis of the detector
        self.b1 = np.array([0, self.size/2.0 - (-self.size/2.0), self.size/2.0 - self.size/2.0])
        self.b2 = np.array([0, -self.size/2.0 - (-self.size/2.0), -self.size/2.0 - self.size/2.0])

        # SIDES

        # create the sympy side planes
        self.sides.append(Polygon3D([a, e, h, d]))
        self.sides.append(Polygon3D([b, f, e, a]))
        self.sides.append(Polygon3D([c, g, f, b]))
        self.sides.append(Polygon3D([d, h, c, g]))

        self.sides[0].polygon_2d = self.sides[0].polygon_2d.buffer(0.002)
        self.sides[1].polygon_2d = self.sides[1].polygon_2d.buffer(0.002)
        self.sides[2].polygon_2d = self.sides[2].polygon_2d.buffer(0.002)
        self.sides[3].polygon_2d = self.sides[3].polygon_2d.buffer(0.002)

        self.vertices.append(a)
        self.vertices.append(b)
        self.vertices.append(c)
        self.vertices.append(d)
        self.vertices.append(e)
        self.vertices.append(f)
        self.vertices.append(g)
        self.vertices.append(h)

    def getVertices(self):

        # detector front_vertices in the world frame
        a = self.vertices[0]
        b = self.vertices[1]
        c = self.vertices[2]
        d = self.vertices[3]
        e = self.vertices[4]
        f = self.vertices[5]
        g = self.vertices[6]
        h = self.vertices[7]

        return [a, b, c, d, e, f, g, h]

    def plotVertices(self):

        [a, b, c, d, e, f, g, h] = self.getVertices()

        xx = [a[0], b[0], c[0], d[0], a[0], e[0], f[0], b[0], f[0], g[0], c[0], g[0], h[0], d[0], h[0], e[0]]
        yy = [a[1], b[1], c[1], d[1], a[1], e[1], f[1], b[1], f[1], g[1], c[1], g[1], h[1], d[1], h[1], e[1]]
        zz = [a[2], b[2], c[2], d[2], a[2], e[2], f[2], b[2], f[2], g[2], c[2], g[2], h[2], d[2], h[2], e[2]]

        return xx, yy, zz

    def plotPixelVertices(self, x, y):

        x = x/256.0
        y = y/256.0

        dpx = 1.0/256.0

        thickness_offset = np.array([self.thickness/2, 0, 0])

        a = self.vertices[0] + self.b1*x + self.b2*y
        b = self.vertices[0] + self.b1*(x+dpx) + self.b2*y
        c = self.vertices[0] + self.b1*(x+dpx) + self.b2*(y+dpx)
        d = self.vertices[0] + self.b1*x + self.b2*(y+dpx)

        e = a - 2*thickness_offset
        f = b - 2*thickness_offset
        g = c - 2*thickness_offset
        h = d - 2*thickness_offset

        xx = [a[0], b[0], c[0], d[0], a[0], e[0], f[0], b[0], f[0], g[0], c[0], g[0], h[0], d[0], h[0], e[0]]
        yy = [a[1], b[1], c[1], d[1], a[1], e[1], f[1], b[1], f[1], g[1], c[1], g[1], h[1], d[1], h[1], e[1]]
        zz = [a[2], b[2], c[2], d[2], a[2], e[2], f[2], b[2], f[2], g[2], c[2], g[2], h[2], d[2], h[2], e[2]]

        return xx, yy, zz

    def getPixelMidPoint(self, x, y):

        x = x/256.0
        y = y/256.0

        dpx = 0.5/256.0

        thickness_offset = np.array([self.thickness/2, 0, 0])

        a = self.vertices[0] + self.b1*(x+dpx) + self.b2*(y+dpx) - thickness_offset

        return a

    def point2pixel(self, point):

        x = (point[1] - self.vertices[0][1])
        y = (point[2] - self.vertices[0][2])

        x = int(m.floor((x/self.b1[1])*256))
        y = int(m.floor((y/self.b2[2])*256))

        return x, y

# #} end of class Detector

class ComptonCamera:

    # #{ comptonScattering()
    
    def comptonScattering(from_point, to_point, energy, material, cs_cross_section, cs_density):
    
        distance = np.linalg.norm(to_point - from_point)
    
        # calculate the probability of the scattering
    
        # draw and decide wether it should happend
        prob_cs = 1.0 - np.exp(-material.electron_density * cs_cross_section * distance)
    
        if random.uniform(0.0, 1.0) > prob_cs:
            return False, 0, 0
    
        # calculate the point of scattering in the detector
        # scattering_point = (from_point + to_point)/2.0
        position_weight = random.uniform(0.0, 1.0)
        scattering_point = position_weight*from_point + (1 - position_weight)*to_point
    
        # calculate the azimuthal and radial angle
        phi = random.uniform(0.0, 2.0*m.pi)
        theta = cs_density[int(m.floor(random.uniform(0.0, len(cs_density))))]
        # theta = 10.0*(m.pi/180.0)
    
        # calculate the point on a unit sphere
        x1 = m.cos(theta)
        y1 = m.sin(theta)*m.sin(phi)
        z1 = m.sin(theta)*m.cos(phi)
    
        v1 = np.array([1.0, 0.0, 0.0])
        v2 = np.array([x1, y1, z1])
        my_axis = np.cross(v1, v2)
    
        original_direction = to_point - from_point
        original_direction = original_direction/np.linalg.norm(original_direction)
    
        my_axis2 = np.cross(v1, original_direction)
        angle2 = geometry.solid_angle.vector_angle(v1, original_direction)
    
        my_quaternion_1 = Quaternion(axis=my_axis2, angle=angle2)
        my_axis = my_quaternion_1.rotate(my_axis)
    
        # print("pes: {}".format(geometry.solid_angle.vector_angle(v1, v2)))
    
        try:
            my_quaternion = Quaternion(axis=my_axis, angle=geometry.solid_angle.vector_angle(v1, v2))
            new_ray_point = scattering_point + my_quaternion.rotate(original_direction)
        except:
            # no rotation should be applied
            new_ray_point = scattering_point + original_direction
            theta = 0.0
    
        # calculate the energy of the new photon
        new_photon_energy = source.energy * physics.comptonRatio(physics.conversions.energy_ev_to_J(source.energy), theta)
        electron_energy = source.energy - new_photon_energy
    
        new_ray = Ray(scattering_point, new_ray_point, new_photon_energy)
    
        return new_ray, electron_energy, theta
    
    # #} end of 

    # #{ sampleDetector()
    
    def sampleDetector(detector):
    
        [a, b, c, d, e, f, g, h] = detector.getVertices()
    
        ab = b-a
        ad = d-a
    
        k1 = random.uniform(1e-2, 1.0-1e-2)
        k2 = random.uniform(1e-2, 1.0-1e-2)
    
        return a + ab*k1 + ad*k2
    
    # #} end of sampleDetector()

    # #{ simulate()
    
    def simulate():
    
       point = sampleDetector(detector_1)
    
       ray = Ray(source_point, point, source.energy)
    
       # intersection with the back side of the 1st detector
       intersect1_second = detector_1.back.intersection(ray)
    
       # no collision with the back face
       if not isinstance(intersect1_second, np.ndarray):
           return
    
           # check intersection with the sides
           for i,side in enumerate(detector_1.sides):
    
               intersect1_second = side.intersection(ray)
    
               if isinstance(intersect1_second, np.ndarray):
                   break
    
       # if the ray came from the oposite direction, discard it
       if np.linalg.norm(intersect1_second - source.position) < np.linalg.norm(point - source.position):
           return
    
       # if there is not a collission with any other facet of the detector
       if not isinstance(intersect1_second, np.ndarray):
           print("! no intersection with the back/side face of the first detector")
           return
    
       # calculate the length of the intersection with the detector
       intersection_len = np.linalg.norm(point - intersect1_second)
    
       # scatter the ray
       scattered_ray, electron_energy, theta = comptonScattering(point, intersect1_second, source.energy, detector_1.material, cs_cross_section, cs_density)
    
       # if not scattering happened, just leave
       if not isinstance(scattered_ray, Ray):
           return
    
       # check the collision with the other detector's front side
       intersect2_first = detector_2.front.intersection(scattered_ray)
    
       # if there is no intersection with the front face, just leave
       if not isinstance(intersect2_first, np.ndarray):
           return
    
       # check the collision with the other detector's back side
       intersect2_second = detector_2.back.intersection(scattered_ray)
    
       # no collision with the back face
       if not isinstance(intersect2_second, np.ndarray):
    
           ## intersection with the sides
           for i,side in enumerate(detector_2.sides):
    
               intersect2_second = side.intersection(scattered_ray)
    
               if isinstance(intersect2_second, np.ndarray):
                   break
    
       # if there is no intersection with the other facets of the detector
       if not isinstance(intersect2_second, np.ndarray):
           print("!! no intersection with the back/side face of the second detector")
           return
    
       # calculate the photo-electric cross section for the scattered photon
       pe_cross_section = [physics.pe_cs_gavrila_pratt_simplified(mat, scattered_ray.energy) for mat in detector_2.material.elements]
       # and the effective thickness of the material for the PE effect
       pe_thickness = np.linalg.norm(intersect2_second - intersect2_first)
    
       prob_pe = 1.0
       for index,cross_section in enumerate(pe_cross_section):
           prob_pe *= np.exp(-detector_2.material.element_quantities[index] * detector_2.material.molecular_density * cross_section * pe_thickness)
       prob_pe = 1.0 - prob_pe
    
       # do a coin toss for the photo-electric effect
       if random.uniform(0.0, 1.0) > prob_pe:
           return
    
       # sample the interraction point in the 2nd detector
       position_weight = random.uniform(0.0, 1.0)
       absorption_point = position_weight*intersect2_first + (1 - position_weight)*intersect2_second
    
       print("Complete compton: p_energy: {}, absorber_thickness: {}, prob_pe: {}".format(scattered_ray.energy, pe_thickness, prob_pe))
    
       # calculate the scattering pixel 
       x, y = detector_1.point2pixel(scattered_ray.rayPoint)
       xs, ys, zs = detector_1.plotPixelVertices(x, y)
       scatterer_mid_point = detector_1.getPixelMidPoint(x, y)
    
       # calculate the absorbtion pixel 
       x, y = detector_2.point2pixel(absorption_point)
       xs, ys, zs = detector_2.plotPixelVertices(x, y)
       absorber_mid_point = detector_2.getPixelMidPoint(x, y)
    
       # #{ cone reconstruction
    
       # reconstruct the tone 
       # add noise to the energies
       if simulate_energy_noise:
           e_noise = random.gauss(0, 7000)
           f_noise = random.gauss(0, 7000)
       else:
           e_noise = 0
           f_noise = 0
    
       # calculate the direction of the cone
       if simulate_pixel_uncertainty:
           cone_origin = scatterer_mid_point
           cone_direction = cone_origin - absorber_mid_point
       else: 
           cone_origin = scattered_ray.rayPoint
           cone_direction = cone_origin - absorption_point
    
       # normalize cone direction
       cone_direction = cone_direction/np.linalg.norm(cone_direction)
    
       # estimate the scattering angle theta
       theta_estimate = physics.getComptonAngle(electron_energy+e_noise, scattered_ray.energy+f_noise)
    
       print("theta_estimate: {}".format(theta_estimate))
    
       # swap the cone, if its angle is > 90 deg
       if theta_estimate > m.pi/2:
           theta_estimate = m.pi - theta_estimate
           cone_direction *= -1.0
    
       print("theta_estimate: {}".format(theta_estimate))
    
       cone = Cone(cone_origin, cone_direction, theta_estimate)
    
       # #} end of cone reconstruction
    
    # #} end of simulate()

    def callbackRadiationSource(self, data):

        rospy.loginfo_once('getting radiation')
        self.radiation_source = data
        self.got_radiation = True

        self.source.activity = self.radiation_source.activity

    def callbackOdometry(self, data):

        rospy.loginfo_once('getting odometry')
        self.odometry = data
        self.got_odometry = True

    def sourceTimer(self, event):

        if not self.got_odometry or not self.got_radiation:
            rospy.loginfo_throttle(1.0, 'waiting for data')
            return

        # transform the source the coordinates of the camera
        source_camera_frame = np.array([self.radiation_source.x - self.odometry.pose.pose.position.x,
                                        self.radiation_source.y - self.odometry.pose.pose.position.y,
                                        self.radiation_source.z - self.odometry.pose.pose.position.z])

        my_quaternion = Quaternion(self.odometry.pose.pose.orientation.w,
                                   self.odometry.pose.pose.orientation.x,
                                   self.odometry.pose.pose.orientation.y,
                                   self.odometry.pose.pose.orientation.z)

        self.source.position  = my_quaternion.rotate(source_camera_frame)

        detector_solid_angle = geometry.solid_angle.quadrilateral_solid_angle(self.a1, self.b1, self.c1, self.d1, self.source.position)
        self.aparent_activity = self.source.activity*(detector_solid_angle/(4*m.pi))
        print("aparent_activity: {}".format(self.aparent_activity))

    def __init__(self):

        rospy.init_node('compton_camera', anonymous=True)

        # define the source and the detector
        self.source = Source(10, 20, np.array([0.0, 0.0, 0.0]))
        self.source_distance = np.linalg.norm(self.source.position)
        self.source_point = self.source.position
        self.detector_1 = Detector(materials.Si, 0.001, np.array([0, 0, 0]))
        self.detector_2 = Detector(materials.CdTe, 0.002, np.array([-0.005, 0, 0]))
        
        # a = np.array([-0.1, 2.0*self.source.position[0], 1.0*self.source.position[2]])
        # b = np.array([-0.1, -2.0*self.source.position[0], 1.0*self.source.position[2]])
        # c = np.array([2.0*self.source.position[0], -2.0*self.source.position[0], 1.0*self.source.position[2]])
        # d = np.array([2.0*self.source.position[0], 2.0*self.source.position[0], 1.0*self.source.position[2]])
        # self.ground_polygon = Polygon3D([a, b, c, d])
        
        [self.a1, self.b1, self.c1, self.d1, e1, f1, g1, h1] = self.detector_1.getVertices()
        [a2, b2, c2, d2, e2, f2, g2, h2] = self.detector_2.getVertices()
        
        # prepare the Compton scattering cross section and probability density for the source's energy
        self.cs_cross_section = physics.comptonCrossSection(physics.conversions.energy_ev_to_J(self.source.energy))
        self.cs_density_indeces, cs_density = physics.cs_distribution_function(self.detector_1.material, self.source.energy)

        # parameters
        # self.pipeline_file = rospy.get_param('~pipeline_path', '/')

        # subscribers
        rospy.Subscriber("~radiation_source_in", RadiationSource, self.callbackRadiationSource, queue_size=1)
        rospy.Subscriber("~odometry_in", Odometry, self.callbackOdometry, queue_size=1)

        # publishers
        # self.publisher_image = rospy.Publisher("~labeled_out", Image, queue_size=1)

        rospy.Timer(rospy.Duration(0.1), self.sourceTimer)

        rospy.spin()

if __name__ == '__main__':
    try:
        compton_camera = ComptonCamera()
    except rospy.ROSInterruptException:
        pass
