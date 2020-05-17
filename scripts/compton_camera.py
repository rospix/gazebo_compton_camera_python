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

# my stuff
import geometry.solid_angle
import photon_attenuation.materials as materials
import photon_attenuation.physics as physics
from geometry.raytracing import Plane, Ray
from geometry.cone import Cone
from geometry.rectangle3d import Rectangle3D

from nav_msgs.msg import Odometry
from gazebo_rad_msgs.msg import RadiationSource

from gazebo_rad_msgs.msg import Cone as ConeMsg

simulate_energy_noise = True
simulate_pixel_uncertainty = True

# simulate_energy_noise = False
# simulate_pixel_uncertainty = False

# #{ roundup()

def roundup(x, to):

    return int(m.ceil(x / to)) * to

# #} end of roundup()

# #{ class Source

class Source:

    def __init__(self, energy, activity, position, id):

        self.energy = energy
        self.activity = activity
        self.position = position
        self.aparent_activity = 0
        self.last_update = rospy.Time.now()
        self.id = id

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
        self.a = np.array([self.thickness/2, -self.size/2.0, -self.size/2.0]) + position
        self.b = np.array([self.thickness/2, self.size/2.0, -self.size/2.0]) + position
        self.c = np.array([self.thickness/2, self.size/2.0, self.size/2.0]) + position
        self.d = np.array([self.thickness/2, -self.size/2.0, self.size/2.0]) + position

        # create the sympy front plane
        self.front = Rectangle3D([self.a, self.b, self.c, self.d])

        # BACK

        # give me the 4 back_vertices of the detector
        self.e = np.array([-self.thickness/2, -self.size/2.0, -self.size/2.0]) + position
        self.f = np.array([-self.thickness/2, self.size/2.0, -self.size/2.0]) + position
        self.g = np.array([-self.thickness/2, self.size/2.0, self.size/2.0]) + position
        self.h = np.array([-self.thickness/2, -self.size/2.0, self.size/2.0]) + position

        # create the sympy back plane
        self.back = Rectangle3D([self.e, self.f, self.g, self.h])

        # orthogonal basis of the detector
        self.b1 = np.array([0, self.size, 0])
        self.b2 = np.array([0, 0, self.size])

        # SIDES

        # create the sympy side planes
        self.sides.append(Rectangle3D([self.e, self.a, self.d, self.h]))
        self.sides.append(Rectangle3D([self.a, self.e, self.f, self.b]))
        self.sides.append(Rectangle3D([self.b, self.f, self.g, self.c]))
        self.sides.append(Rectangle3D([self.c, self.g, self.h, self.d]))

        self.vertices.append(self.a)
        self.vertices.append(self.b)
        self.vertices.append(self.c)
        self.vertices.append(self.d)
        self.vertices.append(self.e)
        self.vertices.append(self.f)
        self.vertices.append(self.g)
        self.vertices.append(self.h)

        self.ab = self.b-self.a
        self.ad = self.d-self.a

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

    # #{ __init__()

    def __init__(self):

        self.is_initialized = False

        rospy.init_node('compton_camera', anonymous=True)

        self.detector_1 = Detector(materials.Si, 0.001, np.array([0, 0, 0]))
        self.detector_2 = Detector(materials.CdTe, 0.002, np.array([-0.005, 0, 0]))

        # a = np.array([-0.1, 2.0*self.source.position[0], 1.0*self.source.position[2]])
        # b = np.array([-0.1, -2.0*self.source.position[0], 1.0*self.source.position[2]])
        # c = np.array([2.0*self.source.position[0], -2.0*self.source.position[0], 1.0*self.source.position[2]])
        # d = np.array([2.0*self.source.position[0], 2.0*self.source.position[0], 1.0*self.source.position[2]])
        # self.ground_polygon = Rectangle3D([a, b, c, d])

        [self.a1, self.b1, self.c1, self.d1, e1, f1, g1, h1] = self.detector_1.getVertices()
        [a2, b2, c2, d2, e2, f2, g2, h2] = self.detector_2.getVertices()

        # parameters
        self.rad_timer_dt = rospy.get_param('~rad_timer_dt')
        self.energy_granularity = rospy.get_param('~energy_granularity')

        self.cs_cross_sections = dict()
        self.cs_densities = dict()

        for i in range(10, 1000, self.energy_granularity):
            rospy.loginfo('[ComptonCamera]: preparing CS crossection for {} kev'.format(i))
            self.cs_cross_sections[i] = physics.comptonCrossSection(physics.conversions.energy_ev_to_J(i*1000.0))

        for i in range(10, 1000, self.energy_granularity):
            rospy.loginfo('[ComptonCamera]: preparing CS density for {} kev'.format(i))
            cs_density_indeces, cs_density = physics.cs_distribution_function(self.detector_1.material, i*1000.0, 0.01, 0.1)
            self.cs_densities[i] = cs_density

        # subscribers
        rospy.Subscriber("~radiation_source_in", RadiationSource, self.callbackRadiationSource, queue_size=1)
        rospy.Subscriber("~odometry_in", Odometry, self.callbackOdometry, queue_size=1)

        self.got_radiation = False
        self.got_odometry = False

        self.rad_sources = []
        self.rad_sources_ids = dict()

        # publishers
        self.publisher_cones = rospy.Publisher("~cone_out", ConeMsg, queue_size=1)

        rospy.Timer(rospy.Duration(self.rad_timer_dt), self.sourceTimer)

        self.is_initialized = True

        rospy.spin()

    # #} end of __init__(self)

    # #{ comptonScattering()

    def comptonScattering(self, from_point, to_point, energy, material, cs_cross_section, cs_density):

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
        new_photon_energy = energy * physics.comptonRatio(physics.conversions.energy_ev_to_J(energy), theta)
        electron_energy = energy - new_photon_energy

        new_ray = Ray(scattering_point, new_ray_point, new_photon_energy)

        return new_ray, electron_energy, theta

    # #} end of

    # #{ sampleDetector()

    def sampleDetector(self, detector):

        k1 = random.uniform(1e-2, 1.0-1e-2)
        k2 = random.uniform(1e-2, 1.0-1e-2)

        return detector.a + detector.ab*k1 + detector.ad*k2

    # #} end of sampleDetector()

    # #{ samplePolygon()

    def samplePolygon(self, facet):

        k1 = random.uniform(1e-2, 1.0-1e-2)
        k2 = random.uniform(1e-2, 1.0-1e-2)

        return facet.zero_point + facet.v1*k1 + facet.v2*k2

    # #} end of sampleDetector()

    # #{ simulate()

    def simulate(self, energy, source_point, facet, cs_cross_section, cs_density):

        point = self.samplePolygon(facet)

        ray_from_source = Ray(source_point, point, energy)
        ray = Ray(point + ray_from_source.rayDirection*0.0000001, point + ray_from_source.rayDirection, energy)

        # intersection with the back side of the 1st detector
        intersect1_second = self.detector_1.back.intersection(ray)

        # no collision with the back face, check the front face
        if not isinstance(intersect1_second, np.ndarray):
            intersect1_second = self.detector_1.front.intersection(ray)

        # no collision with the back/front face, check the side facets
        if not isinstance(intersect1_second, np.ndarray):

            # check intersection with the sides
            for i,side in enumerate(self.detector_1.sides):

                intersect1_second = side.intersection(ray)

                if isinstance(intersect1_second, np.ndarray):
                    break

        # # if the ray came from the oposite direction, discard it
        # if np.linalg.norm(intersect1_second - source_point) < np.linalg.norm(point - source_point):
        #     return

        # if there is not a collission with any other facet of the detector
        if not isinstance(intersect1_second, np.ndarray):
            print("! no intersection with the back/side face of the first detector")
            return

        # calculate the length of the intersection with the detector
        intersection_len = np.linalg.norm(point - intersect1_second)

        # scatter the ray
        scattered_ray, electron_energy, theta = self.comptonScattering(point, intersect1_second, energy, self.detector_1.material, cs_cross_section, cs_density)

        # if not scattering happened, just leave
        if not isinstance(scattered_ray, Ray):
            return

        # check the collision with the other detector's front side
        intersect2_first = self.detector_2.front.intersection(scattered_ray)

        # if there is no intersection with the front face, just leave
        if not isinstance(intersect2_first, np.ndarray):
            return

        # check the collision with the other detector's back side
        intersect2_second = self.detector_2.back.intersection(scattered_ray)

        # no collision with the back face
        if not isinstance(intersect2_second, np.ndarray):

            ## intersection with the sides
            for i,side in enumerate(self.detector_2.sides):

                intersect2_second = side.intersection(scattered_ray)

                if isinstance(intersect2_second, np.ndarray):
                    break

        # if there is no intersection with the other facets of the detector
        if not isinstance(intersect2_second, np.ndarray):
            print("!! no intersection with the back/side face of the second detector")
            return

        # calculate the photo-electric cross section for the scattered photon
        pe_cross_section = [physics.pe_cs_gavrila_pratt_simplified(mat, scattered_ray.energy) for mat in self.detector_2.material.elements]
        # and the effective thickness of the material for the PE effect
        pe_thickness = np.linalg.norm(intersect2_second - intersect2_first)

        prob_pe = 1.0
        for index,cross_section in enumerate(pe_cross_section):
            prob_pe *= np.exp(-self.detector_2.material.element_quantities[index] * self.detector_2.material.molecular_density * cross_section * pe_thickness)
        prob_pe = 1.0 - prob_pe

        # do a coin toss for the photo-electric effect
        if random.uniform(0.0, 1.0) > prob_pe:
            return

        # sample the interraction point in the 2nd detector
        position_weight = random.uniform(0.0, 1.0)
        absorption_point = position_weight*intersect2_first + (1 - position_weight)*intersect2_second

        print("Complete compton: p_energy: {}, absorber_thickness: {}, prob_pe: {}".format(scattered_ray.energy, pe_thickness, prob_pe))

        # calculate the scattering pixel
        x, y = self.detector_1.point2pixel(scattered_ray.rayPoint)
        xs, ys, zs = self.detector_1.plotPixelVertices(x, y)
        scatterer_mid_point = self.detector_1.getPixelMidPoint(x, y)

        # calculate the absorbtion pixel
        x, y = self.detector_2.point2pixel(absorption_point)
        xs, ys, zs = self.detector_2.plotPixelVertices(x, y)
        absorber_mid_point = self.detector_2.getPixelMidPoint(x, y)

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

        # swap the cone, if its angle is > 90 deg
        if theta_estimate > m.pi/2:
            theta_estimate = m.pi - theta_estimate
            cone_direction *= -1.0

        # cone_direction = np.array([1, 1, 1])
        # theta_estimate = m.pi/4.0

        # transform the cone to world coordinates
        # cone = Cone(cone_origin, cone_direction, theta_estimate)
        cone_direction = self.quaternion_d2w.rotate(cone_direction)

        cone_origin = self.quaternion_d2w.rotate(cone_origin)
        cone_origin = cone_origin + np.array([self.odometry.pose.pose.position.x, self.odometry.pose.pose.position.y, self.odometry.pose.pose.position.z])

        v1 = np.array([1.0, 0.0, 0.0])
        axis = np.cross(v1, cone_direction)
        angle = geometry.solid_angle.vector_angle(v1, cone_direction)
        cone_orientation = Quaternion(axis=axis, angle=angle)

        # publish the cone to ROS
        cone_msg = ConeMsg()
        cone_msg.pose.position.x = cone_origin[0]
        cone_msg.pose.position.y = cone_origin[1]
        cone_msg.pose.position.z = cone_origin[2]
        cone_msg.pose.orientation.x = cone_orientation[1]
        cone_msg.pose.orientation.y = cone_orientation[2]
        cone_msg.pose.orientation.z = cone_orientation[3]
        cone_msg.pose.orientation.w = cone_orientation[0]
        cone_msg.direction.x = cone_direction[0]
        cone_msg.direction.y = cone_direction[1]
        cone_msg.direction.z = cone_direction[2]
        cone_msg.angle = theta_estimate

        self.publisher_cones.publish(cone_msg)

        # #} end of cone reconstruction

    # #} end of simulate()

    # #{ callbackRadiationSource()

    def callbackRadiationSource(self, data):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[ComptonCamera]: getting radiation')
        self.got_radiation = True

        if data.id in self.rad_sources_ids:

            idx = self.rad_sources_ids.get(data.id)
            self.rad_sources[idx-1].position = np.array([data.world_pos.x, data.world_pos.y, data.world_pos.z])
            self.rad_sources[idx-1].last_update = rospy.Time.now()

        else:

            rospy.loginfo('[ComptonCamera]: registering new source with ID {}'.format(data.id))

            if not data.material in materials.radiation_sources:
                rospy.logerr('the material {} is not in the database of radiation sources'.format(data.material))
            else:

                new_source = Source(materials.radiation_sources[data.material].photon_energy, data.activity, np.array([data.world_pos.x, data.world_pos.y, data.world_pos.z]), data.id)
                self.rad_sources.append(new_source)
                self.rad_sources_ids[data.id] = len(self.rad_sources)

    # #} end of callbackRadiationSource()

    # #{ callbackOdometry()

    def callbackOdometry(self, data):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[ComptonCamera]: getting odometry')
        self.odometry = data
        self.got_odometry = True

        # world2drone
        self.quaternion_d2w = Quaternion(self.odometry.pose.pose.orientation.w,
                                         self.odometry.pose.pose.orientation.x,
                                         self.odometry.pose.pose.orientation.y,
                                         self.odometry.pose.pose.orientation.z)

        # drone2world
        self.quaternion_w2d = self.quaternion_d2w.inverse

    # #} end of callbackOdometry()

    # #{ sourceTimer()

    def sourceTimer(self, event):

        if not self.is_initialized:
            return

        if not self.got_odometry or not self.got_radiation:
            rospy.loginfo_throttle(1.0, '[ComptonCamera]: waiting for data')
            return

        # for each of the sources in the scene
        for source_idx,source in enumerate(self.rad_sources):

            if (rospy.Time.now() - source.last_update).to_sec() > 1.0:
                continue

            # transform the source the coordinates of the camera
            source_camera_frame = np.array([source.position[0] - self.odometry.pose.pose.position.x,
                                            source.position[1] - self.odometry.pose.pose.position.y,
                                            source.position[2] - self.odometry.pose.pose.position.z])

            source_position_in_local = self.quaternion_w2d.rotate(source_camera_frame)

            # rospy.loginfo_throttle(1.0, 'Source in FCU: {} {} {}'.format(source_position_in_local[0], source_position_in_local[1], source_position_in_local[2]))

            # for each facet of the detector
            facets = [self.detector_1.front, self.detector_1.sides[0], self.detector_1.sides[1], self.detector_1.sides[2], self.detector_1.sides[3]]

            for facet_idx,facet in enumerate(facets):

                # print("facet_idx: {}, facet_idx, facet.plane.planeNormal: {}".format(facet_idx, facet.plane.planeNormal))
                # print("source_position_in_local: {}".format(source_position_in_local))

                # check if the facet is in the right orientation towards the source
                if facet.plane.planeNormal.dot(source_position_in_local) <= 0:
                    continue

                # calculate the apparent activity of the source
                facet_solid_angle = geometry.solid_angle.quadrilateral_solid_angle(facet.points_3d[0], facet.points_3d[1], facet.points_3d[2], facet.points_3d[3], source_position_in_local)
                aparent_activity = source.activity*(facet_solid_angle/(4*m.pi))

                # how many particle are we shooting towards the rectangle
                n_particles = aparent_activity*self.rad_timer_dt

                # round up the energy
                energy_roundedup = roundup(source.energy/1000.0, float(self.energy_granularity))

                # find the pre-calculated cross section for the energy
                cs_cross_section = self.cs_cross_sections[energy_roundedup]
                cs_density = self.cs_densities[energy_roundedup]

                # simulate n particles
                time_start = rospy.Time.now()

                # simulate n_particles
                if n_particles > 1:

                  for i in range(0, int(n_particles)):
                  
                      self.simulate(source.energy, source_position_in_local, facet, cs_cross_section, cs_density)
                  
                      duration = (rospy.Time.now() - time_start).to_sec()
                      if duration > 0.01/len(self.rad_sources):
                          break

                # the number of particles is lower than 1
                # lets do a coin toss to determine whether a particle will be simulated
                else:
                    if random.uniform(0.0, 1.0) < n_particles:

                        rospy.loginfo('[ComptonCamera]: particle simualted by a coin toss') 
                        self.simulate(source.energy, source_position_in_local, facet, cs_cross_section, cs_density)

                duration = (rospy.Time.now() - time_start).to_sec()

                rospy.loginfo('[ComptonCamera]: aparent_activity of the source {} towards the facet {} is {} ({}), duration={} s'.format(source.id, facet_idx, aparent_activity, n_particles, duration))

    # #} end of callbackOdometry()

if __name__ == '__main__':
    try:
        compton_camera = ComptonCamera()
    except rospy.ROSInterruptException:
        pass
