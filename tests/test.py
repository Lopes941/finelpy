import finelpy as fp
import numpy as np
import scipy as sp


from time import perf_counter as time

from matplotlib import pyplot as plt


# Creating Geometry
Lx = 80
Ly = 50
nx = 160*5
ny = 50
t = 1

O = (0,0,0)

Rec = fp.geometry.Rectangle((Lx,Ly),O)

# Creating a new material
properties = {
    fp.material.MaterialProperties.YOUNGS_MOD: 210e9,
    fp.material.MaterialProperties.RHO: 7850,
    fp.material.MaterialProperties.POISSON: 0.3
}

steel = fp.material.create_material("Steel").add_property(properties)

el = fp.element.create_element(
        fp.element.ShapeType.TRI3, 
        fp.element.ModelType.PLANE_STRUCTURAL,
        fp.material.ConstitutiveType.PLANE_STRESS, 
        steel)

mesh_gen = fp.mesh.RectangularMesh(Rec, el)
mesh_gen.set_grid(nx,ny)
mesh = mesh_gen.build()

an_gen = fp.analysis.AnalysisBuilder(mesh)
an_gen.add_boundary_condition(fp.analysis.DOFType.UX,Rec.left_side,0)
an_gen.add_boundary_condition(fp.analysis.DOFType.UY,Rec.left_side,0)

forc_node = mesh.find_node((40,50))
an_gen.add_force(fp.analysis.DOFType.UY,forc_node,-1e3)
analysis = an_gen.build()

solver_dir = fp.solver.StaticSolver(analysis, fp.solver.SolverType.Direct)
res = solver_dir.solve()

p, val = res.get_max(fp.results.ResultData.SIGMA_VONMISES,internal_pts=10)
print(f"Maximum Von mises at {p}, with value of {val:0.2e}")

p, val = res.get_max(fp.results.ResultData.SIGMA_XX,internal_pts=10)
print(f"Maximum Sigma_x at {p}, with value of {val:0.2e}")


res.plot_deformed(10e7,True)

analysis.destroy()
plt.show()