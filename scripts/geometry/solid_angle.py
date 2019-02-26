import numpy as np
import math as m

def normalize(a):

    return a/np.linalg.norm(a)

def haversine(a):

    return (1.0 - m.cos(a))/2.0

def inv_haversine(a):

    return 2*m.asin(m.sqrt(a))

def spherical_angle(a, b, c):

    return inv_haversine((haversine(c) - haversine(a-b))/(m.sin(a)*m.sin(b)))

def vector_angle(a, b):

    return m.acos(np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b)))

def spherical_triangle_area(a, b, c):

    # arc langts
    ab = vector_angle(a, b)
    bc = vector_angle(b, c)
    ca = vector_angle(c, a)

    # spherical angles
    A = spherical_angle(ca, ab, bc)
    B = spherical_angle(ab, bc, ca)
    C = spherical_angle(bc, ca, ab)
    
    return A + B + C - m.pi

# a, b, c, d are the vertices of a quadrilateral in 3D, CCW or CW
# z is the center of the field of view
def quadrilateral_solid_angle(a, b, c, d, z):

    # project the points to a unit sphere
    a = normalize(a - z)
    b = normalize(b - z)
    c = normalize(c - z)
    d = normalize(d - z)

    # calculate the area of its two triangles
    tri_1 = spherical_triangle_area(a, b, c)
    tri_2 = spherical_triangle_area(c, d, a)

    return tri_1 + tri_2

# # testing
# size = 0.01408
# distance = 100.0

# a = np.array([size, 0, distance])
# b = np.array([0, size, distance])
# c = np.array([-size, 0, distance])
# d = np.array([0, -size, distance])
# z = np.array([0, 0, 0])

# area = (4*m.pi)/quadrilateral_solid_angle(a, b, c, d, z)
# print("area: 1/{}".format(area))
