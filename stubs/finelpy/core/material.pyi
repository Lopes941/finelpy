"""
Material submodule
"""
from __future__ import annotations
import typing
__all__: list[str] = ['IConstitutiveModel', 'Material', 'MaterialCatalogue', 'MaterialProperties']
class IConstitutiveModel:
    pass
class Material:
    def __init__(self) -> None:
        ...
    @typing.overload
    def add_property(self, arg0: MaterialProperties, arg1: typing.SupportsFloat) -> Material:
        """
        Add or update a material property
        """
    @typing.overload
    def add_property(self, arg0: dict) -> Material:
        """
        Add or update a material property
        """
    def get_property(self, arg0: MaterialProperties) -> float:
        """
        Get a material property value
        """
    @property
    def A(self) -> float:
        ...
    @property
    def E(self) -> float:
        ...
    @property
    def IZZ(self) -> float:
        ...
    @property
    def nu(self) -> float:
        ...
    @property
    def rho(self) -> float:
        ...
class MaterialCatalogue:
    @staticmethod
    def get_catalogue() -> MaterialCatalogue:
        """
        Get the singleton MaterialCatalogue
        """
    def create_material(self, arg0: str) -> Material:
        """
                       Create a rectangle with given dimensions and origin.
        
                       Parameters
                       ----------
                       name : string
                            name of the material.
                       
                       Returns
                       ----------
                       Material
                            Reference to created material.
        """
    def get_material(self, arg0: str) -> Material:
        """
        Get an existing material by name
        """
class MaterialProperties:
    """
    Members:
    
      RHO
    
      YOUNGS_MOD
    
      YOUNGS_MOD_XX
    
      YOUNGS_MOD_YY
    
      YOUNGS_MOD_ZZ
    
      POISSON
    
      POISSON_XY
    
      POISSON_XZ
    
      POISSON_YZ
    
      THERMAL_COND_X
    
      THERMAL_COND_Y
    
      THERMAL_COND_Z
    
      IZZ
    
      A
    """
    A: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.A: 14>
    IZZ: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.IZZ: 13>
    POISSON: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.POISSON: 5>
    POISSON_XY: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.POISSON_XY: 6>
    POISSON_XZ: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.POISSON_XZ: 7>
    POISSON_YZ: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.POISSON_YZ: 8>
    RHO: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.RHO: 0>
    THERMAL_COND_X: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.THERMAL_COND_X: 10>
    THERMAL_COND_Y: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.THERMAL_COND_Y: 11>
    THERMAL_COND_Z: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.THERMAL_COND_Z: 12>
    YOUNGS_MOD: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.YOUNGS_MOD: 1>
    YOUNGS_MOD_XX: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.YOUNGS_MOD_XX: 2>
    YOUNGS_MOD_YY: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.YOUNGS_MOD_YY: 3>
    YOUNGS_MOD_ZZ: typing.ClassVar[MaterialProperties]  # value = <MaterialProperties.YOUNGS_MOD_ZZ: 4>
    __members__: typing.ClassVar[dict[str, MaterialProperties]]  # value = {'RHO': <MaterialProperties.RHO: 0>, 'YOUNGS_MOD': <MaterialProperties.YOUNGS_MOD: 1>, 'YOUNGS_MOD_XX': <MaterialProperties.YOUNGS_MOD_XX: 2>, 'YOUNGS_MOD_YY': <MaterialProperties.YOUNGS_MOD_YY: 3>, 'YOUNGS_MOD_ZZ': <MaterialProperties.YOUNGS_MOD_ZZ: 4>, 'POISSON': <MaterialProperties.POISSON: 5>, 'POISSON_XY': <MaterialProperties.POISSON_XY: 6>, 'POISSON_XZ': <MaterialProperties.POISSON_XZ: 7>, 'POISSON_YZ': <MaterialProperties.POISSON_YZ: 8>, 'THERMAL_COND_X': <MaterialProperties.THERMAL_COND_X: 10>, 'THERMAL_COND_Y': <MaterialProperties.THERMAL_COND_Y: 11>, 'THERMAL_COND_Z': <MaterialProperties.THERMAL_COND_Z: 12>, 'IZZ': <MaterialProperties.IZZ: 13>, 'A': <MaterialProperties.A: 14>}
    def __eq__(self, other: typing.Any) -> bool:
        ...
    def __getstate__(self) -> int:
        ...
    def __hash__(self) -> int:
        ...
    def __index__(self) -> int:
        ...
    def __init__(self, value: typing.SupportsInt) -> None:
        ...
    def __int__(self) -> int:
        ...
    def __ne__(self, other: typing.Any) -> bool:
        ...
    def __repr__(self) -> str:
        ...
    def __setstate__(self, state: typing.SupportsInt) -> None:
        ...
    def __str__(self) -> str:
        ...
    @property
    def name(self) -> str:
        ...
    @property
    def value(self) -> int:
        ...
