import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from ..core.material import MaterialCatalogue, ConstitutiveType
from ..core.material import Material, ConstitutiveModel
from ..core.material import MaterialProperties


def create_material(name: str) -> Material:
    """
        Create material with given name.

        Parameters
        -----------
        name: str
            Name of material.

        Returns
        -----------
        material: Material
            Created material.

    """

    return MaterialCatalogue.get_catalogue().create_material(name)

def material_error(name: str):
    raise RuntimeError(f"Must define {name} function in pyConstitutiveModel class")

class pyConstitutiveModel(ConstitutiveModel):

    def __init__(self, material: Material):
        super().__init__()
        self.material: Material = material

    def is_linear(self):
        return True
    
    def D(self, loc=None, disp=None):
        material_error("D")

    def get_property(self, property: ConstitutiveModel):
        return self.material.get_property(property)