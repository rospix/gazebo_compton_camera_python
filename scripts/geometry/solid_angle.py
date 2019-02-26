import numpy as np
import math as m

def normalize(a):

    return a/np.linalg.norm(a)

def haversine(a):

    return (1.0 - m.cos(a))/2.0

def inv_haversine(h):

    return 2*m.asin(m.sqrt(h))

# a, b, c: the side lenghts in a spherical triangle
def spherical_angle(a, b, c):

    # # cosine theorem
    # spherical_angle = m.acos((m.pow(c, 2) - m.pow(a, 2) - m.pow(b, 2))/(-2*a*b))

    spherical_angle = inv_haversine((haversine(c) - haversine(a-b))/(m.sin(a)*m.sin(b)))

    return spherical_angle

def vector_angle(a, b):

    vector_angle = m.acos(np.dot(a, b)/(np.linalg.norm(a)*np.linalg.norm(b)))

    return vector_angle 

def spherical_triangle_area(a, b, c):

    # arc langts
    ab = vector_angle(a, b)
    bc = vector_angle(b, c)
    ca = vector_angle(c, a)

    area = 0

    # use the normal triangle area when the triangle is veeeery small
    if ab < 1e-3 and bc < 1e-3 and ca < 1e-3:

        p = (ab + bc + ca)/2.0
            
        area = m.sqrt(p*(p - ab)*(p - bc)*(p - ca))

    # use the hoversine formula when the sphericallity could influance the result
    else:

        # spherical angles
        A = spherical_angle(ca, ab, bc)
        B = spherical_angle(ab, bc, ca)
        C = spherical_angle(bc, ca, ab)
        
        area = A + B + C - m.pi

    return area

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
