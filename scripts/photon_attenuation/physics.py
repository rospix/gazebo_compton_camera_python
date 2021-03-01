import math as m
import numpy as np

import photon_attenuation.constants as constants
import photon_attenuation.conversions as conversions

# #{ comptonRatio()

def comptonRatio(energy, theta):

    return 1.0 / (1.0 + (energy/(constants.me*m.pow(constants.c, 2)))*(1.0 - m.cos(theta)))

# #} end of comptonRatio()

# #{ getComptonAngle()

def getComptonAngle(Ee, Ef):

    if Ee < 0:
        Ee = 0

    if Ef < 0:
        Ef = 0

    E0 = Ee + Ef

    Ee_J = conversions.energy_ev_to_J(Ee)
    Ef_J = conversions.energy_ev_to_J(Ef)
    E0_J = conversions.energy_ev_to_J(E0)

    try:
        # angle = m.acos(1.0 - constants.me*m.pow(constants.c, 2.0)*(Ee_J/(E0_J*(E0_J-Ee_J)))) # Dan's
        angle = m.acos(1.0 + constants.me*m.pow(constants.c, 2.0)*(1.0/E0_J - 1.0/Ef_J)) # mine
    except Exception as e:
        return m.pi

    return angle

# #} end of getComptonAngle()

# #{ comptonDiffCrossSection()

def comptonDiffCrossSection(energy, theta):

    P = comptonRatio(energy, theta)

    # # energy of the scattered photon
    # E_p = energy * P
    # # energy of the scattered photon in in keV
    # E_p_keV = 0.001*energy_J_to_eV(E_p)
    # # energy of the recoil electron
    # E_e = energy * (1 - P)
    # # energy of the scattered photon in in keV
    # E_e_keV = 0.001*energy_J_to_eV(E_e)

    return 0.5 * m.pow(P, 2) * m.pow(constants.r_e, 2) * (P + 1.0/P - m.pow(m.sin(theta), 2))

# #} end of comptonDiffCrossSection()

# #{ comptonCrossSection()

def comptonCrossSection(energy):

    total_cross_section = 0

    step = conversions.deg2rad(0.1) # [rad]

    for theta in np.arange(-m.pi, m.pi+step, step):

        # Klein-Nishina formula
        diff_cross_section = comptonDiffCrossSection(energy, theta)

        # target solid angle
        solid_angle = m.pi * m.sin(m.fabs(theta)) * step

        total_cross_section += diff_cross_section * solid_angle

    return total_cross_section

# #} end of comptonCrossSection()

# #{ pe_cs_gavrila_pratt_simplified()

def pe_cs_gavrila_pratt_simplified(material, energy):

    e_rest_mass_energy = 511000.0
    k = energy/e_rest_mass_energy

    return (16.0/3.0)*m.sqrt(2.0)*m.pi*m.pow(constants.r_e, 2)*m.pow(constants.alpha, 4)*(m.pow(material.atomic_number, 5)/m.pow(k, 3.5))
    # return conversions.barn2m2(3.0e12*(m.pow(material.atomic_number, 4)/m.pow(energy, 3.5)))

# #} end of pe_cs_gavrila_pratt_simplified()

# #{ pe_cs_gavrila_pratt()

def pe_cs_gavrila_pratt(material, energy):

    electron_rest_mass = conversions.energy_J_to_eV(constants.me*m.pow(constants.c*100, 2))

    constant = ((4.0*m.pi*m.pow(m.e, 4))/(m.pow(electron_rest_mass, 2)))*(1.0/constants.alpha)
    constant = 1.367e-22

    return constant*m.pow(constants.alpha*material.atomic_number, 5)*(electron_rest_mass/energy)*0.0001

# #} end of pe_cs_gavrila_pratt()

# #{ pe_cs_hubell_k_shell()

# photoelectric effect coefficients
pe_a = [1.6268e-9, 1.5274e-9, 1.1330e-9, -9.12e-11]
pe_b = [-2.683e-12, -5.110e-13, -2.177e-12, 0.0]
pe_c = [4.173e-2, 1.027e-2, 2.013e-2, 0.0]
pe_p = [1.0, 2.0, 3.5, 4.0]

def pe_cs_hubell_k_shell(material, energy):

    summ = 0
    for i in range(0, 4):
        summ += ((pe_a[i] + pe_b[i]*material.atomic_number)/(1 + pe_c[i]*material.atomic_number))*m.pow(energy/1000000.0, -pe_p[i])

    return conversions.barn2m2(m.pow(material.atomic_number, 5)*summ)

# #} end of pe_cs_hubell_k_shell()

# #{ pe_cs_hubell()

def pe_cs_hubell(material, energy):

    boundary = 1500000.0

    if energy < boundary:
        return peeDavisson(material, energy)
    else:
        davisson_boundary = peeDavisson(material, boundary)
        hubell_boundary = pe_cs_hubell_k_shell(material, boundary)

        return (davisson_boundary/hubell_boundary)*pe_cs_hubell_k_shell(material, energy)

# #} end of pe_cs_hubell()

# #{ cs_interaction_depth()

def cs_interaction_depth(material, energy, thickness=0.01, depth_granularity=0.000001):

    distribution = np.zeros((int(m.floor(thickness/depth_granularity))))
    density = np.zeros((int(m.floor(thickness/depth_granularity))))
    indeces = np.zeros((int(m.floor(thickness/depth_granularity))))

    total_cross_section = comptonCrossSection(conversions.energy_ev_to_J(energy))

    n_steps = len(distribution)

    previous_prob = 0.0

    sub_thickness = depth_granularity

    prob_slice = 1.0 - np.exp(-material.electron_density * total_cross_section * depth_granularity)

    for i in range(0, n_steps):

        prob = 1.0 - np.exp(-material.electron_density * total_cross_section * sub_thickness)

        sub_thickness += depth_granularity

        distribution[i] = 1.0 - (1.0-prob_slice)*(1.0-previous_prob)
        indeces[i] = sub_thickness

        previous_prob = prob

    for i in range(1, n_steps):

        density[i-1] = distribution[i] - distribution[i-1]

    density[-1] = density[-2]

    return indeces, distribution, density

# #} end of cs_interaction_depth()

# #{ cs_distribution_function()

def cs_distribution_function(material, energy, perc_granularity=0.001, angle_step=0.01):

    energy_J = conversions.energy_ev_to_J(energy)

    angle_step = conversions.deg2rad(angle_step) # [rad]

    prev = 0

    sigma_normalized = []
    sigma_normalized_cumulative = []
    abs_prob = []
    angles = []
    klein_nishina = []

    for theta in np.arange(-m.pi, m.pi+angle_step, angle_step):

      # Klein-Nishina formula
      compton_diff_cross_section = comptonDiffCrossSection(energy_J, theta)
      klein_nishina.append(compton_diff_cross_section)

      # target solid angle
      omega_1 = m.pi * m.sin(m.fabs(theta)) * angle_step

      prob = compton_diff_cross_section * omega_1

      sigma_normalized.append(prob)

      sigma_normalized_cumulative.append(prev + prob)

      prev = prev + prob

      angles.append(theta)

    total = sum(sigma_normalized)

    sigma_normalized_cumulative = [x/total for x in sigma_normalized_cumulative]

    distribution = np.zeros((int(m.floor(1.0/perc_granularity))))
    indeces = np.zeros((int(m.floor(1.0/perc_granularity))))

    n_steps = len(distribution)

    for i in range(0, n_steps):

        indeces[i] = (1.0/n_steps)*i

        prob_lim = perc_granularity*i

        for index,prob in enumerate(sigma_normalized_cumulative):

            if prob > prob_lim:

                distribution[i] = -m.pi + ((m.pi*2.0)/len(sigma_normalized_cumulative))*index

                break

    return  indeces, distribution

# #} end of cs_distribution_function()

# #{ air_absorption_coefficient(energy):

# def air_absorption_coefficient(energy):

# 1.00000e-03  3.606e+03
# 1.50000e-03  1.191e+03
# 2.00000e-03  5.279e+02
# 3.00000e-03  1.625e+02
# 3.20290e-03  1.340e+02
# 3.20290e-03  1.485e+02
# 4.00000e-03  7.788e+01
# 5.00000e-03  4.027e+01
# 6.00000e-03  2.341e+01
# 8.00000e-03  9.921e+00
# 1.00000e-02  5.120e+00
# 1.50000e-02  1.614e+00
# 2.00000e-02  7.779e-01
# 3.00000e-02  3.538e-01
# 4.00000e-02  2.485e-01
# 5.00000e-02  2.080e-01
# 6.00000e-02  1.875e-01
# 8.00000e-02  1.662e-01
# 1.00000e-01  1.541e-01
# 1.50000e-01  1.356e-01
# 2.00000e-01  1.233e-01
# 3.00000e-01  1.067e-01
# 4.00000e-01  9.549e-02
# 5.00000e-01  8.712e-02
# 6.00000e-01  8.055e-02
# 8.00000e-01  7.074e-02
# 1.00000e+00  6.358e-02
# 1.25000e+00  5.687e-02
# 1.50000e+00  5.175e-02
# 2.00000e+00  4.447e-02
# 3.00000e+00  3.581e-02
# 4.00000e+00  3.079e-02
# 5.00000e+00  2.751e-02
# 6.00000e+00  2.522e-02
# 8.00000e+00  2.225e-02
# 1.00000e+01  2.045e-02
# 1.50000e+01  1.810e-02
# 2.00000e+01  1.705e-02

# #} end of air_absorption_coefficient(energy):
