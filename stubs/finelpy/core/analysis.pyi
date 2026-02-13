"""
Analysis submodule
"""
from __future__ import annotations
import collections.abc
import finelpy.core.geometry
import finelpy.core.mesh
import numpy
import numpy.typing
import typing
__all__: list[str] = ['Analysis', 'AnalysisBuilder', 'DOFType', 'IInterpolationScheme', 'InterpolationParameters', 'InterpolationScheme']
class Analysis:
    def Kg(self) -> numpy.ndarray:
        ...
    def Mg(self) -> numpy.ndarray:
        ...
    def destroy(self) -> None:
        ...
    def fg(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        ...
    def set_interpolation(self, arg0: InterpolationScheme) -> None:
        ...
    def update_interpolation(self, arg0: InterpolationParameters, arg1: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def update_pseudo_density(self, arg0: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def update_pseudo_density(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> None:
        ...
    @property
    def ID(self) -> numpy.ndarray:
        ...
    @property
    def bc_dofs(self) -> numpy.typing.NDArray[numpy.int32]:
        ...
    @property
    def free_dofs(self) -> numpy.typing.NDArray[numpy.int32]:
        ...
    @property
    def interpolation_scheme(self) -> InterpolationScheme:
        ...
    @property
    def mesh(self) -> finelpy.core.mesh.Mesh:
        ...
    @property
    def num_bc_dofs(self) -> int:
        ...
    @property
    def num_free_dofs(self) -> int:
        ...
    @property
    def num_total_dofs(self) -> int:
        ...
class AnalysisBuilder:
    def __init__(self, mesh: finelpy.core.mesh.Mesh) -> None:
        ...
    @typing.overload
    def add_boundary_condition(self, arg0: DOFType, arg1: typing.SupportsInt, arg2: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def add_boundary_condition(self, arg0: DOFType, arg1: collections.abc.Sequence[typing.SupportsInt], arg2: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def add_boundary_condition(self, arg0: DOFType, arg1: finelpy.core.geometry.IGeometry, arg2: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def add_force(self, arg0: DOFType, arg1: typing.SupportsInt, arg2: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def add_force(self, arg0: DOFType, arg1: collections.abc.Sequence[typing.SupportsInt], arg2: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def add_force(self, dof_type: DOFType, domain: finelpy.core.geometry.IGeometry, value: typing.SupportsFloat, number_of_integration: typing.SupportsInt = 10) -> None:
        ...
    @typing.overload
    def add_force(self, dof_type: DOFType, domain: finelpy.core.geometry.IGeometry, function: collections.abc.Callable[[Tuple[float, float] or Tuple[float, float, float]], float], number_of_integration: typing.SupportsInt = 10) -> None:
        ...
    def build(self) -> Analysis:
        ...
class DOFType:
    """
    Members:
    
      UX
    
      UY
    
      UZ
    
      THETAX
    
      THETAY
    
      THETAZ
    
      T
    
      P
    """
    P: typing.ClassVar[DOFType]  # value = <DOFType.P: 7>
    T: typing.ClassVar[DOFType]  # value = <DOFType.T: 6>
    THETAX: typing.ClassVar[DOFType]  # value = <DOFType.THETAX: 3>
    THETAY: typing.ClassVar[DOFType]  # value = <DOFType.THETAY: 4>
    THETAZ: typing.ClassVar[DOFType]  # value = <DOFType.THETAZ: 5>
    UX: typing.ClassVar[DOFType]  # value = <DOFType.UX: 0>
    UY: typing.ClassVar[DOFType]  # value = <DOFType.UY: 1>
    UZ: typing.ClassVar[DOFType]  # value = <DOFType.UZ: 2>
    __members__: typing.ClassVar[dict[str, DOFType]]  # value = {'UX': <DOFType.UX: 0>, 'UY': <DOFType.UY: 1>, 'UZ': <DOFType.UZ: 2>, 'THETAX': <DOFType.THETAX: 3>, 'THETAY': <DOFType.THETAY: 4>, 'THETAZ': <DOFType.THETAZ: 5>, 'T': <DOFType.T: 6>, 'P': <DOFType.P: 7>}
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
class IInterpolationScheme:
    pass
class InterpolationParameters:
    """
    Members:
    
      P_EXPONENT
    
      X_MIN
    
      BETA
    
      THRESHOLD
    """
    BETA: typing.ClassVar[InterpolationParameters]  # value = <InterpolationParameters.BETA: 2>
    P_EXPONENT: typing.ClassVar[InterpolationParameters]  # value = <InterpolationParameters.P_EXPONENT: 0>
    THRESHOLD: typing.ClassVar[InterpolationParameters]  # value = <InterpolationParameters.THRESHOLD: 3>
    X_MIN: typing.ClassVar[InterpolationParameters]  # value = <InterpolationParameters.X_MIN: 1>
    __members__: typing.ClassVar[dict[str, InterpolationParameters]]  # value = {'P_EXPONENT': <InterpolationParameters.P_EXPONENT: 0>, 'X_MIN': <InterpolationParameters.X_MIN: 1>, 'BETA': <InterpolationParameters.BETA: 2>, 'THRESHOLD': <InterpolationParameters.THRESHOLD: 3>}
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
class InterpolationScheme:
    """
    Members:
    
      NONE
    
      SIMP
    """
    NONE: typing.ClassVar[InterpolationScheme]  # value = <InterpolationScheme.NONE: 0>
    SIMP: typing.ClassVar[InterpolationScheme]  # value = <InterpolationScheme.SIMP: 1>
    __members__: typing.ClassVar[dict[str, InterpolationScheme]]  # value = {'NONE': <InterpolationScheme.NONE: 0>, 'SIMP': <InterpolationScheme.SIMP: 1>}
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
