"""
Geometry submodule
"""
from __future__ import annotations
import numpy
import typing
__all__: list[str] = ['AreaType', 'IArea', 'IGeometry', 'Line', 'Rectangle']
class AreaType:
    """
    Members:
    
      POLYGON
    
      RECTANGLE
    """
    POLYGON: typing.ClassVar[AreaType]  # value = <AreaType.POLYGON: 0>
    RECTANGLE: typing.ClassVar[AreaType]  # value = <AreaType.RECTANGLE: 1>
    __members__: typing.ClassVar[dict[str, AreaType]]  # value = {'POLYGON': <AreaType.POLYGON: 0>, 'RECTANGLE': <AreaType.RECTANGLE: 1>}
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
class IArea(IGeometry):
    def __init__(self, arg0: numpy.ndarray[float, shape=(n,3)]) -> None:
        ...
    def name(self) -> AreaType:
        ...
    @property
    def lines(self) -> list[Line]:
        ...
    @property
    def nodes(self) -> numpy.ndarray:
        ...
    @property
    def num_vertices(self) -> int:
        ...
    @property
    def vertices(self) -> numpy.ndarray[float, shape=(n,3)]:
        ...
class IGeometry:
    def is_inside(self, arg0: Tuple[float, float] or Tuple[float, float, float]) -> bool:
        ...
    @property
    def nodes(self) -> numpy.ndarray:
        ...
class Line(IGeometry):
    @typing.overload
    def __init__(self, point_1: Tuple[float, float] or Tuple[float, float, float], point_2: Tuple[float, float] or Tuple[float, float, float]) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: typing.SupportsFloat) -> None:
        ...
    @typing.overload
    def __init__(self, arg0: typing.SupportsFloat, arg1: typing.SupportsFloat) -> None:
        ...
    @property
    def len(self) -> float:
        ...
    @property
    def p1(self) -> Tuple[float, float] or Tuple[float, float, float]:
        ...
    @property
    def p2(self) -> Tuple[float, float] or Tuple[float, float, float]:
        ...
class Rectangle(IArea):
    @staticmethod
    def static_name() -> AreaType:
        ...
    def __init__(self, dimensions: Tuple[float, float] or Tuple[float, float, float], origin: Tuple[float, float] or Tuple[float, float, float] = (0, 0)) -> None:
        """
                    Create a rectangle with given dimensions and origin.
        
                    Parameters
                    ----------
                    dimensions : list of float, length 2 or 3
                        Dimensions lx, ly and t (thickness).
                    origin: list of float, length 2 or 3, optional
                        Optional origin for first point.
        """
    def plot(self, facecolor = 'skyblue', edgecolor = 'darkblue'):
        ...
    @property
    def dimensions(self) -> Tuple[float, float] or Tuple[float, float, float]:
        ...
    @property
    def left_side(self) -> Line:
        ...
    @property
    def lower_side(self) -> Line:
        ...
    @property
    def origin(self) -> Tuple[float, float] or Tuple[float, float, float]:
        ...
    @property
    def right_side(self) -> Line:
        ...
    @property
    def upper_side(self) -> Line:
        ...
