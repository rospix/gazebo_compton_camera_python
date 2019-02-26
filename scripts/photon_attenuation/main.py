#!/bin/python

import math as m
import matplotlib
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.ticker import ScalarFormatter

import os

import materials
import conversions
import constants
import physics

# latex in pyplot
from matplotlib import rc
rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)

# energy of the incoming photon
E_0_keV = [0.0027, 60, 511, 1460, 10000]
E_0_J = [conversions.energy_ev_to_J(E * 1000) for E in E_0_keV] # [J]

# scatterer thickness
scatterer_z = 0.001
scatterer_material = materials.Si

# absorber thickness
absorber_z = 0.001
absorber_material = materials.CdTe

a_list = []
angles = []
klein_nishina = []
sigma_normalized_normalized = []
sigma_normalized = []
sigma_normalized_cumulative = []
abs_prob = []
total_cross_section = []
total_area = 0

step = conversions.deg2rad(0.1) # [rad]

print("scatterer {} electron density: {} 1/cm^3 \n".format(scatterer_material.name, scatterer_material.electron_density/(100*100*100)))
print("absorber {} electron density: {} 1/cm^3 \n".format(absorber_material.name, absorber_material.electron_density/(100*100*100)))

for energy_idx,energy in enumerate(E_0_J):

    sublist1 = []
    klein_nishina.append(sublist1)
    sublist2 = []
    sigma_normalized_normalized.append(sublist2)
    sublist3 = []
    sigma_normalized.append(sublist3)
    sublist4 = []
    abs_prob.append(sublist4)
    sublist5 = []
    sigma_normalized_cumulative.append(sublist5)
    total_cross_section.append(physics.comptonCrossSection(energy))

    prev = 0

    for theta in np.arange(-m.pi, m.pi+step, step):

      # Klein-Nishina formula
      compton_diff_cross_section = physics.comptonDiffCrossSection(energy, theta)
      klein_nishina[energy_idx].append(compton_diff_cross_section)
      
      # target solid angle
      omega_1 = m.pi * m.sin(m.fabs(theta)) * step

      # did not work well for thick materials
      # prob = CdTe_e_density * compton_diff_cross_section * omega_1 * scatterer_z
      # prob = 1 - np.exp(-CdTe_e_density * compton_diff_cross_section * omega_1 * scatterer_z)

      prob = compton_diff_cross_section * omega_1

      sigma_normalized[energy_idx].append(prob)

      sigma_normalized_cumulative[energy_idx].append(prev + prob)

      prev = prev + prob

      if energy_idx == 0:
          angles.append(theta)
          total_area += omega_1

# normalize sigma_normalized
for energy_idx,energy in enumerate(E_0_J):

    total = sum(sigma_normalized[energy_idx])
    sigma_normalized[energy_idx] = [x/total for x in sigma_normalized[energy_idx]]
    sigma_normalized_cumulative[energy_idx] = [x/total for x in sigma_normalized_cumulative[energy_idx]]

# convert to abs. prob
for energy_idx,energy in enumerate(E_0_J):

    total_prob = 1 - np.exp(-scatterer_material.electron_density * total_cross_section[energy_idx] * scatterer_z)
    abs_prob[energy_idx] = [x*total_prob for x in sigma_normalized[energy_idx]]

sigma_marginalized = [[x/total_cross_section[sublist_idx] for x in sublist] for sublist_idx,sublist in enumerate(sigma_normalized)]
# sigma_normalized_normalized = [[x/total_cross_section[sublist_idx] for x in sublist] for sublist_idx,sublist in enumerate(klein_nishina)]
# sigma_normalized_normalized = [[x for x in sublist] for sublist_idx,sublist in enumerate(sigma_normalized)]

# #{ test prints

for energy_idx,energy in enumerate(E_0_J):
    summ = 0
    for idx,value in enumerate(sigma_normalized[energy_idx]):
        summ += value
    print("prob_sum for {} keV: {}".format(conversions.energy_J_to_eV(energy)/1000, summ))

for energy_idx,energy in enumerate(E_0_keV):
    print("")
    print("total_cross_section[energy_idx]: {0:1.2f} re^2".format(total_cross_section[energy_idx]/m.pow(constants.r_e, 2)))

    prob = 1 - np.exp(-scatterer_material.electron_density * total_cross_section[energy_idx] * scatterer_z)
    # prob = scatterer_material.electron_density * total_cross_section[energy_idx] * scatterer_z

    print("{0:2.1f}% of photons are scattered for {1:6.1f} keV".format(prob*100, energy))

# #} end of test prints

print("")
print("total area: {0:2.3f} sr".format(total_area))

prob_pe_scatterer = []
prob_cs_scatterer = []

prob_pe_absorber = []
prob_cs_absorber = []

prob_absorber_attenuation = []
prob_scatterer_attenuation = []

pe_energies = []

# #{ calculating attenuations

for e in range(1, 1000, 1): # over keV

    pe_energies.append(e)

    pe_cross_section_scatterer = [physics.pe_cs_gavrila_pratt_simplified(mat, e*1000.0) for mat in scatterer_material.elements]
    cs_cross_section_scatterer = physics.comptonCrossSection(conversions.energy_ev_to_J(e*1000.0))

    pe_cross_section_absorber = [physics.pe_cs_gavrila_pratt_simplified(mat, e*1000.0) for mat in absorber_material.elements]
    cs_cross_section_absorber = physics.comptonCrossSection(conversions.energy_ev_to_J(e*1000.0))

    prod = 1
    for index,cross_section in enumerate(pe_cross_section_scatterer):
        prod *= np.exp(-scatterer_material.element_quantities[index]*scatterer_material.molecular_density * cross_section * scatterer_z)
    prob = 1 - prod
    prob_pe_scatterer.append(prob)

    prob = 1 - np.exp(-scatterer_material.electron_density * cs_cross_section_scatterer * scatterer_z)
    prob_cs_scatterer.append(prob)

    prod = 1
    for index,cross_section in enumerate(pe_cross_section_absorber):
        prod *= np.exp(-absorber_material.element_quantities[index]*absorber_material.molecular_density * cross_section * absorber_z)
    prob = 1 - prod
    prob_pe_absorber.append(prob)

    prob = 1 - np.exp(-absorber_material.electron_density * cs_cross_section_absorber * absorber_z)
    prob_cs_absorber.append(prob)

    prob_absorber_attenuation = [(1 - (1-x[0])*(1-x[1])) for x in zip(prob_cs_absorber, prob_pe_absorber)]
    prob_scatterer_attenuation = [(1 - (1-x[0])*(1-x[1])) for x in zip(prob_cs_scatterer, prob_pe_scatterer)]

# #} end of calculating photon attenuations

# #{ plot_everything()
    
def plot_everything(*args):

    multiplot = False
    plot_klein_nishina = False
    plot_attenuations = True
    plot_compton_distribution = False

    # #{ if plot_klein_nishina:
    
    if plot_klein_nishina:
    
      fig = plt.figure(1)
      fig.canvas.set_window_title("Klein-Nishina")
      if multiplot:
          ax = plt.subplot(141, projection='polar')
      else:
          ax = plt.subplot(111, projection='polar')
      for energy_idx,energy in enumerate(E_0_J):
        ax.plot(angles, klein_nishina[energy_idx], label="{} keV".format(E_0_keV[energy_idx]))
      ax.legend()
      ax.grid(True)
      plt.title("Compton scattering diff. cross section for $\heta \in [0, \pi]$".format())
      plt.savefig("klein_nishina_1.png", bbox_inches="tight")
    
      if multiplot:
          ax = plt.subplot(142, projection='polar')
      else:
          fig = plt.figure(2)
          ax = plt.subplot(111, projection='polar')
      for energy_idx,energy in enumerate(E_0_J):
        ax.plot(angles, abs_prob[energy_idx], label="{} keV".format(E_0_keV[energy_idx]))
      fig.canvas.set_window_title("Compton posterior")
      ax.grid(True)
      ax.legend()
      plt.title('Posterior prob. of scattering by a radial angle $\Theta \in [0, \pi]$'.format())
      plt.savefig("klein_nishina_2.png", bbox_inches="tight")
    
      if multiplot:
          ax = plt.subplot(143, projection='polar')
      else:
          fig = plt.figure(3)
          ax = plt.subplot(111, projection='polar')
      for energy_idx,energy in enumerate(E_0_J):
        ax.plot(angles, sigma_normalized[energy_idx], label="{} keV".format(E_0_keV[energy_idx]))
      fig.canvas.set_window_title("Compton likelihood")
      ax.grid(True)
      ax.legend()
      plt.title('The likelihood of scattering by a radial angle $\Theta$, {} mm {}'.format(scatterer_z*1000, scatterer_material.name))
      plt.savefig("klein_nishina_3.png", bbox_inches="tight")
    
    # #} end of plot

    # #{ if plot_attenuations:
    
    if plot_attenuations:
    
      if multiplot:
          fig = plt.figure(10)
          ax = plt.subplot(211)
      else:
          fig = plt.figure(10)
          ax = plt.subplot(111)
      plt.yscale('log')
      plt.xscale('log')
      for axis in [ax.xaxis, ax.yaxis]:
          axis.set_major_formatter(ScalarFormatter())
      fig.canvas.set_window_title("Photon-attenuation scatterer")
      ax.plot(pe_energies, prob_pe_scatterer, label="Photoelectric effect prob., {}, {} mm".format(scatterer_material.name, scatterer_z/1000.0))
      ax.plot(pe_energies, prob_cs_scatterer, label="Compton scattering prob., {}, {} mm".format(scatterer_material.name, scatterer_z/1000.0))
      ax.plot(pe_energies, prob_scatterer_attenuation, label="Total attenutaion by PE and CS, {}, {} mm".format(scatterer_material.name, scatterer_z/1000.0), linestyle="dashed")
      ax.set_xlabel("Photon energy [keV]")
      ax.set_ylabel("Probability [-]")
      ax.grid(True)
      ax.legend()
      plt.savefig("scatterer_attenuation.png", bbox_inches="tight")
    
      if multiplot:
          ax = plt.subplot(212)
      else:
          fig = plt.figure(11)
          ax = plt.subplot(111)
      plt.yscale('log')
      plt.xscale('log')
      for axis in [ax.xaxis, ax.yaxis]:
          axis.set_major_formatter(ScalarFormatter())
      fig.canvas.set_window_title("Photon-attenuation absorber")
      ax.plot(pe_energies, prob_pe_absorber, label="Photoelectric effect prob., {}, {} mm".format(absorber_material.name, absorber_z/1000.0))
      ax.plot(pe_energies, prob_cs_absorber, label="Compton scattering prob., {}, {} mm".format(absorber_material.name, absorber_z/1000.0))
      ax.plot(pe_energies, prob_absorber_attenuation, label="Total attenutaion by PE and CS, {}, {} mm".format(absorber_material.name, absorber_z/1000.0), linestyle="dashed")
      ax.set_xlabel("Photon energy [keV]")
      ax.set_ylabel("Probability [-]")
      ax.legend()
      ax.grid(True)
      plt.savefig("absorber_attenuation.png", bbox_inches="tight")
    
    # #} end of if plot_attenuations:

    # #{ if plot_compton_distribution:
    
    if plot_compton_distribution:
    
        fig = plt.figure(12)
        ax = plt.subplot(111)
        for axis in [ax.xaxis, ax.yaxis]:
            axis.set_major_formatter(ScalarFormatter())
        fig.canvas.set_window_title("Cumulative Compton likelihood")
        for energy_idx,energy in enumerate(E_0_J):
            ax.plot(angles, sigma_normalized_cumulative[energy_idx], label="{} keV".format(E_0_keV[energy_idx]))
        ax.set_xlabel("Angle [rad]")
        ax.set_ylabel("cumulative prob [-]")
        ax.grid(True)
        ax.legend()
        plt.savefig("compton_cumulative_distributiion.png", bbox_inches="tight")
    
        fig = plt.figure(13)
        ax = plt.subplot(111)
        for axis in [ax.xaxis, ax.yaxis]:
            axis.set_major_formatter(ScalarFormatter())
        fig.canvas.set_window_title("Compton distribution")
        # for energy_idx,energy in enumerate(E_0_J):
        for energy_idx,energy in enumerate(E_0_J):
            indeces, distribution = physics.cs_distribution_function(scatterer_material, E_0_keV[energy_idx]*1000.0)
            ax.plot(indeces, distribution, label="{} keV".format(E_0_keV[energy_idx]))
        ax.set_xlabel("Prob [-]")
        ax.set_ylabel("Angle [rad]")
        ax.grid(True)
        ax.legend()
        plt.savefig("compton_distribution.png", bbox_inches="tight")
    
        fig = plt.figure(14)
        ax = plt.subplot(111)
        for axis in [ax.xaxis, ax.yaxis]:
            axis.set_major_formatter(ScalarFormatter())
        fig.canvas.set_window_title("Compton stopping depth distribution")
        # for energy_idx,energy in enumerate(E_0_J):
        for energy_idx,energy in enumerate(E_0_J):
            indeces, distribution, density = physics.cs_interaction_depth(scatterer_material, E_0_keV[energy_idx]*1000.0)
            ax.plot(indeces, distribution, label="{} keV".format(E_0_keV[energy_idx]))
        ax.set_xlabel("Depth [m]")
        ax.set_ylabel("Prob [-]")
        ax.grid(True)
        ax.legend()
        plt.savefig("compton_depth.png", bbox_inches="tight")
    
    # #} end of if plot_compton_distribution:

    plt.show()
    
# #} end of plot_everything()

pid = os.fork()
if pid == 0:
    plot_everything()
