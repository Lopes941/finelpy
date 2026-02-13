"""
Mesh submodule
"""
from __future__ import annotations
import collections.abc
import finelpy.core.element
import finelpy.core.geometry
import numpy
import numpy.typing
import typing
__all__: list[str] = ['FrameMesh', 'LineMesh', 'Mesh', 'MeshBuilder', 'RectangularMesh']
class FrameMesh(MeshBuilder):
    def __init__(self, element: finelpy.core.element.Element, node_coord: numpy.ndarray[float, shape=(n,3)], inci: collections.abc.Sequence[collections.abc.Sequence[typing.SupportsInt]]) -> None:
        ...
class LineMesh(MeshBuilder):
    def __init__(self, line: finelpy.core.geometry.Line, element: finelpy.core.element.Element) -> None:
        ...
    def create_from_element_num(self, arg0: typing.SupportsInt) -> None:
        ...
    def create_from_element_size(self, arg0: typing.SupportsFloat) -> None:
        ...
class Mesh:
    def find_element(self, arg0: typing.Annotated[numpy.typing.ArrayLike, numpy.float64, "[m, 1]"]) -> int:
        ...
    def find_node(self, arg0: Tuple[float, float] or Tuple[float, float, float]) -> int:
        ...
    @property
    def centers(self) -> numpy.ndarray[float, shape=(n,3)]:
        ...
    @property
    def element_nodes(self) -> list:
        ...
    @property
    def elements(self) -> list:
        ...
    @property
    def nodes(self) -> numpy.ndarray[float, shape=(n,3)]:
        ...
    @property
    def number_of_elements(self) -> int:
        ...
    @property
    def number_of_nodes(self) -> int:
        ...
    @property
    def ravel_elements(self) -> list[list[list[float]]]:
        ...
class MeshBuilder:
    def build(self) -> Mesh:
        ...
class RectangularMesh(MeshBuilder):
    def __init__(self, domain: finelpy.core.geometry.IArea, element: finelpy.core.element.Element) -> None:
        ...
    def set_element_size(self, arg0: typing.SupportsFloat, arg1: typing.SupportsFloat) -> None:
        ...
    def set_grid(self, arg0: typing.SupportsInt, arg1: typing.SupportsInt) -> None:
        ...
