import finelpy as fp
import numpy as np
import scipy as sp


from time import perf_counter as time

from matplotlib import pyplot as plt

# Creating Geometry
Lx = 10
Ly = 10
nx = 10
ny = 10
t = 1

O = (0,0,0)

Rec = fp.geometry.Rectangle((Lx,Ly),O)

# Creating a new material
properties = {
    fp.material.MaterialProperties.THERMAL_COND: 1
}

mat = fp.material.create_material("Mat").add_property(properties)

el = fp.element.create_element(
        fp.element.ShapeType.TRI3, 
        fp.element.ModelType.THERMAL_CONDUCTION_2D,
        fp.material.ConstitutiveType.THERMAL_CONDUCTION_2D, 
        mat)


mesh_gen = fp.mesh.RectangularMesh(Rec, el)
mesh_gen.set_grid(nx,ny)
mesh = mesh_gen.build()

an_gen = fp.analysis.AnalysisBuilder(mesh)

an_gen.add_boundary_condition(fp.analysis.DOFType.T,Rec.left_side,0)
an_gen.add_force(fp.analysis.DOFType.T,Rec.right_side,1,2)
# an_gen.add_force(fp.analysis.DOFType.T,Rec,1,2)
analysis = an_gen.build()


solver_dir = fp.solver.StaticSolver(analysis, fp.solver.SolverType.Direct)

res = solver_dir.solve()

fig, ax = res.plot_2D_grid(fp.results.ResultData.T,internal_pts=10)

fig, ax = res.plot_2D_grid(fp.results.ResultData.ABS_Q,internal_pts=10)

analysis.destroy()
plt.show()