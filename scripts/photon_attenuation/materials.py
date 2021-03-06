import photon_attenuation.constants as constants

class Element:

    def __init__(self, name, density, atomic_number, molar_mass):

        self.name = name
        self.density = density
        self.atomic_number = atomic_number
        self.molar_mass = molar_mass

        atoms_in_kg = constants.N_a / molar_mass
        self.electron_density = atoms_in_kg * atomic_number * density
        self.atomic_density = atoms_in_kg * density

class RadiationSource:

    def __init__(self, name, atomic_number, activity_per_gr, photon_energy):

        self.name = name
        self.atomic_number = atomic_number
        self.activity_per_gr = activity_per_gr
        self.photon_energy = photon_energy

class Material:

    def __init__(self, name, elements, element_quantities, density, molar_mass):

        self.name = name
        self.density = density
        self.molar_mass = molar_mass
        self.elements = elements
        self.element_quantities = element_quantities

        molecules_in_kg = constants.N_a / molar_mass
        self.electron_density = molecules_in_kg * sum([element[0].atomic_number*element[1] for element in zip(self.elements, self.element_quantities)]) * density
        self.molecular_density = molecules_in_kg * density

Si_element = Element("Si", 2320.0, 14, 0.02808550)
Cd_element = Element("Cd", 8650.0, 48, 0.112411)
Na_element = Element("Cd", 968.0, 11, 0.022989770)
I_element = Element("I", 4933.0, 53, 0.253808940)
Te_element = Element("Te", 5850.0, 52, 0.12760)

radiation_sources = dict()

Cs_137 = RadiationSource("cs137", 137, 3.215e12, 661000)
radiation_sources[Cs_137.name] = Cs_137
Am241 = RadiationSource("am241", 241, 127e9, 59000)
radiation_sources[Am241.name] = Am241

Si = Material("Si", [Si_element], [1], 2320.0, 0.02808550)
Cd = Material("Cd", [Cd_element], [1], 8650.0, 0.112411)
Na = Material("Na", [Na_element], [1], 968.0, 0.022989770)
I = Material("I", [I_element], [1], 4933.0, 0.253808940)
Te = Material("Te", [Te_element], [1], 5850.0, 0.12760)
CdTe = Material("CdTe", [Cd_element, Te_element], [1, 1], 5850.0, 0.2400110)
NaI = Material("NaI", [Na_element, I_element], [1, 1], 3670.0, 0.149894239)
