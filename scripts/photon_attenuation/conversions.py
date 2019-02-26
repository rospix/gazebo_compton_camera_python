import math as m
import constants

# return energy in Jouls
def wavelength_to_J(wavelength):

    return (constants.h * constants.c) / wavelength

# return energy in Jouls
def wavelength_um_to_ev(wavelength):

    return 1.2398 / wavelength

# return energy in eV
def energy_J_to_eV(energy):

    return 6.242e18 * energy

# return energy in eV
def energy_ev_to_J(energy):

    return energy / 6.242e18

def deg2rad(deg):

    return deg*(m.pi / 180.0)

def rad2deg(rad):

    return rad*(180.0 / m.pi)

def barn2m2(barn):

    return barn*constants.barn

def m22barn(m2):

    return m2/constants.barn
