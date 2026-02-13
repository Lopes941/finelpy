"""
Solver submodule
"""
from __future__ import annotations
import finelpy.core.analysis
import finelpy.core.element
import numpy
import numpy.typing
import typing
__all__: list[str] = ['ResultData', 'SolverType', 'StaticResult', 'StaticSolver']
class ResultData:
    """
    Members:
    
      UX
    
      UY
    
      UZ
    
      THETAX
    
      THETAY
    
      THETAZ
    
      ABS_U
    
      EPSILON_XX
    
      EPSILON_YY
    
      EPSILON_ZZ
    
      EPSILON_XY
    
      EPSILON_XZ
    
      EPSILON_YZ
    
      SIGMA_XX
    
      SIGMA_YY
    
      SIGMA_ZZ
    
      SIGMA_XY
    
      SIGMA_XZ
    
      SIGMA_YZ
    
      SIGMA_VONMISES
    
      NX
    
      VY
    
      MZ
    """
    ABS_U: typing.ClassVar[ResultData]  # value = <ResultData.ABS_U: 6>
    EPSILON_XX: typing.ClassVar[ResultData]  # value = <ResultData.EPSILON_XX: 7>
    EPSILON_XY: typing.ClassVar[ResultData]  # value = <ResultData.EPSILON_XY: 10>
    EPSILON_XZ: typing.ClassVar[ResultData]  # value = <ResultData.EPSILON_XZ: 11>
    EPSILON_YY: typing.ClassVar[ResultData]  # value = <ResultData.EPSILON_YY: 8>
    EPSILON_YZ: typing.ClassVar[ResultData]  # value = <ResultData.EPSILON_YZ: 12>
    EPSILON_ZZ: typing.ClassVar[ResultData]  # value = <ResultData.EPSILON_ZZ: 9>
    MZ: typing.ClassVar[ResultData]  # value = <ResultData.MZ: 22>
    NX: typing.ClassVar[ResultData]  # value = <ResultData.NX: 20>
    SIGMA_VONMISES: typing.ClassVar[ResultData]  # value = <ResultData.SIGMA_VONMISES: 19>
    SIGMA_XX: typing.ClassVar[ResultData]  # value = <ResultData.SIGMA_XX: 13>
    SIGMA_XY: typing.ClassVar[ResultData]  # value = <ResultData.SIGMA_XY: 16>
    SIGMA_XZ: typing.ClassVar[ResultData]  # value = <ResultData.SIGMA_XZ: 17>
    SIGMA_YY: typing.ClassVar[ResultData]  # value = <ResultData.SIGMA_YY: 14>
    SIGMA_YZ: typing.ClassVar[ResultData]  # value = <ResultData.SIGMA_YZ: 18>
    SIGMA_ZZ: typing.ClassVar[ResultData]  # value = <ResultData.SIGMA_ZZ: 15>
    THETAX: typing.ClassVar[ResultData]  # value = <ResultData.THETAX: 3>
    THETAY: typing.ClassVar[ResultData]  # value = <ResultData.THETAY: 4>
    THETAZ: typing.ClassVar[ResultData]  # value = <ResultData.THETAZ: 5>
    UX: typing.ClassVar[ResultData]  # value = <ResultData.UX: 0>
    UY: typing.ClassVar[ResultData]  # value = <ResultData.UY: 1>
    UZ: typing.ClassVar[ResultData]  # value = <ResultData.UZ: 2>
    VY: typing.ClassVar[ResultData]  # value = <ResultData.VY: 21>
    __members__: typing.ClassVar[dict[str, ResultData]]  # value = {'UX': <ResultData.UX: 0>, 'UY': <ResultData.UY: 1>, 'UZ': <ResultData.UZ: 2>, 'THETAX': <ResultData.THETAX: 3>, 'THETAY': <ResultData.THETAY: 4>, 'THETAZ': <ResultData.THETAZ: 5>, 'ABS_U': <ResultData.ABS_U: 6>, 'EPSILON_XX': <ResultData.EPSILON_XX: 7>, 'EPSILON_YY': <ResultData.EPSILON_YY: 8>, 'EPSILON_ZZ': <ResultData.EPSILON_ZZ: 9>, 'EPSILON_XY': <ResultData.EPSILON_XY: 10>, 'EPSILON_XZ': <ResultData.EPSILON_XZ: 11>, 'EPSILON_YZ': <ResultData.EPSILON_YZ: 12>, 'SIGMA_XX': <ResultData.SIGMA_XX: 13>, 'SIGMA_YY': <ResultData.SIGMA_YY: 14>, 'SIGMA_ZZ': <ResultData.SIGMA_ZZ: 15>, 'SIGMA_XY': <ResultData.SIGMA_XY: 16>, 'SIGMA_XZ': <ResultData.SIGMA_XZ: 17>, 'SIGMA_YZ': <ResultData.SIGMA_YZ: 18>, 'SIGMA_VONMISES': <ResultData.SIGMA_VONMISES: 19>, 'NX': <ResultData.NX: 20>, 'VY': <ResultData.VY: 21>, 'MZ': <ResultData.MZ: 22>}
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
class SolverType:
    """
    Members:
    
      Direct
    
      Iterative
    """
    Direct: typing.ClassVar[SolverType]  # value = <SolverType.Direct: 0>
    Iterative: typing.ClassVar[SolverType]  # value = <SolverType.Iterative: 1>
    __members__: typing.ClassVar[dict[str, SolverType]]  # value = {'Direct': <SolverType.Direct: 0>, 'Iterative': <SolverType.Iterative: 1>}
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
class StaticResult:
    def compliance(self) -> float:
        ...
    def compliance_sensitivity(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        ...
    def element_mean(self, result_id: ResultData, gauss_pts: typing.SupportsInt = 10) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        ...
    def get_max(self, result_id: ResultData, internal_pts: typing.SupportsInt = 10) -> float:
        ...
    def get_min(self, result_id: ResultData, internal_pts: typing.SupportsInt = 10) -> float:
        ...
    def get_points(self, internal_pts: typing.SupportsInt = 10) -> numpy.ndarray[float, shape=(n,3)]:
        ...
    def grid_data(self, result_id: ResultData, internal_pts: typing.SupportsInt = 10) -> tuple[numpy.ndarray[float, shape=(n,3)], numpy.ndarray[float, shape=(n,)]]:
        ...
    @property
    def analysis(self) -> finelpy.core.analysis.Analysis:
        ...
    @property
    def elements(self) -> list[finelpy.core.element.Element]:
        ...
    @property
    def nodes(self) -> numpy.ndarray[float, shape=(n,3)]:
        ...
    @property
    def u(self) -> typing.Annotated[numpy.typing.NDArray[numpy.float64], "[m, 1]"]:
        ...
class StaticSolver:
    @typing.overload
    def __init__(self, analysis: finelpy.core.analysis.Analysis) -> None:
        ...
    @typing.overload
    def __init__(self, analysis: finelpy.core.analysis.Analysis, solver_type: SolverType) -> None:
        ...
    def solve(self) -> StaticResult:
        ...
