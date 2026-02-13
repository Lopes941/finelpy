"""
finite element module
"""
from __future__ import annotations
from . import analysis
from . import element
from . import geometry
from . import material
from . import mesh
from . import solver
__all__: list[str] = ['analysis', 'element', 'geometry', 'material', 'mesh', 'petsc_enabled', 'solver', 'vtk_enabled']
def petsc_enabled() -> bool:
    ...
def vtk_enabled() -> bool:
    ...
