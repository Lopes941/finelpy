from __future__ import annotations
import finelpy.core.material
from finelpy.core.material import Material
from finelpy.core.material import MaterialCatalogue
from finelpy.core.material import MaterialProperties
from matplotlib import patches
from matplotlib import pyplot as plt
import numpy as np
__all__: list[str] = ['Material', 'MaterialCatalogue', 'MaterialProperties', 'create_material', 'np', 'patches', 'plt']
def create_material(name: str) -> finelpy.core.material.Material:
    ...
