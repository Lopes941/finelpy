import numpy as np

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from ..core.solver import StaticResult, ResultData
from ..core import vtk_enabled

from ..utils import add_method

@add_method(StaticResult)
def plot_data1D(self,
            id: ResultData,
            internal_pts: int=10,
            show_nodes: bool=False,
            mult=1):

    import matplotlib.pyplot as plt 
    import matplotlib.tri as tri

    pdata, cdata = self.grid_data(id,internal_pts)
    if mult != 1:
        cdata*=mult
    xdata = pdata[:,0]

    fig, ax = plt.subplots()
    ax.plot(xdata,cdata)
        
    if show_nodes:
        xnode = self.analysis.mesh.nodes[:,0]
        node_data = np.interp(xnode,xdata,cdata)
        ax.scatter(xnode,node_data,color='black')

    return ax, xdata

@add_method(StaticResult)
def plot_2D_grid(self,
                id: ResultData,
                internal_pts=1,
                colormap='viridis',
                show_edges: bool=False,
                show_nodes: bool=False,
                scale: float = 1):
    
    import matplotlib.colors as colors

    pdata, cdata = self.grid_data(id,internal_pts)

    x = np.empty(pdata.shape[0])
    y = np.empty(pdata.shape[0])

    for i, pi in enumerate(pdata):
        xi, yi, _ = pi
        x[i] = xi
        y[i] = yi

    fig = plt.figure()
    ax = fig.subplots()
    sc = ax.scatter(x,y,c=cdata, cmap=colormap, norm=colors.PowerNorm(gamma=scale))

    ax.autoscale()
    ax.set_aspect("equal")
    fig.colorbar(sc)

    return fig, ax

@add_method(StaticResult)
def plot_deformed(self,
                scale: float = 1,
                show_original: bool=False,
                show_edges = False,
                fig = None,
                ax = None):

        from matplotlib.collections import PolyCollection

        if (ax is None) or (fig is None):
            fig, ax = plt.subplots()
        else:
            ax.clear()

        if show_edges:
            edge_color = "blue"
        else:
            edge_color = "face"

        if show_original:
            fig, ax = self.analysis.mesh.plot_mesh2D(fig=fig, ax=ax, alpha=0.3)


        polygons = self.ravel_displaced_elements(scale)

        pc = PolyCollection(polygons,facecolors="gray",edgecolors=edge_color)
        
        ax.add_collection(pc)
        ax.autoscale()
        ax.set_aspect("equal")
        ax.set_axis_off()

        return fig, ax


# if(not vtk_enabled()):

#     @add_method(StaticResult)
#     def plot_2D_grid(self,
#                     id: ResultData,
#                     internal_pts=10,
#                     show_edges: bool=False,
#                     show_nodes: bool=False):
        
#         import vtk

#         pdata, cdata = self.grid_data(id,internal_pts)

#         vtk_points = vtk.vtkPoints()
#         vtk_values = vtk.vtkFloatArray()
#         vtk_values.SetName("Values")


#         for i in range(pdata.shape[0]):
#             pt = pdata[i,:]
#             vtk_points.InsertNextPoint(pt)
#             vtk_values.InsertNextValue(cdata[i])

#         # Create a vtkPolyData to store points
#         polydata = vtk.vtkPolyData()
#         polydata.SetPoints(vtk_points)
#         polydata.GetPointData().SetScalars(vtk_values)

#         # Create a glyph (sphere) for each point
#         sphere_source = vtk.vtkSphereSource()
#         sphere_source.SetRadius(0.1)  # Base radius

#         glyph = vtk.vtkGlyph3D()
#         glyph.SetSourceConnection(sphere_source.GetOutputPort())
#         glyph.SetInputData(polydata)
#         glyph.SetColorModeToColorByScalar()  # Use scalar values for color
#         glyph.SetScaleModeToScaleByScalar()  # Optionally scale spheres by values
#         glyph.Update()

#         # Mapper and Actor
#         mapper = vtk.vtkPolyDataMapper()
#         mapper.SetInputConnection(glyph.GetOutputPort())
#         mapper.SetScalarRange(np.min(cdata), np.max(cdata))  # Map values to colors

#         actor = vtk.vtkActor()
#         actor.SetMapper(mapper)

#         # Renderer, RenderWindow, Interactor
#         renderer = vtk.vtkRenderer()
#         renderer.AddActor(actor)
#         renderer.SetBackground(0.2, 0.3, 0.4)  # Dark blue background

#         render_window = vtk.vtkRenderWindow()
#         render_window.AddRenderer(renderer)
#         render_window.SetSize(800, 600)

#         interactor = vtk.vtkRenderWindowInteractor()
#         interactor.SetRenderWindow(render_window)

#         # Start visualization
#         render_window.Render()
#         interactor.Start()