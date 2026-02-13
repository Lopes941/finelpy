import finelpy as fp
import numpy as np
import scipy as sp


from time import perf_counter as time

from matplotlib import pyplot as plt


# Creating Geometry
Lx = 80
Ly = 50
nx = 80
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
        fp.element.ShapeType.QUAD4, 
        fp.element.ModelType.PLANE_STRUCTURAL,
        fp.element.ConstitutiveType.PLANE_STRESS, 
        steel)


mesh_gen = fp.mesh.RectangularMesh(Rec, el)
mesh_gen.set_grid(nx,ny)
mesh = mesh_gen.build()



an_gen = fp.analysis.AnalysisBuilder(mesh)
an_gen.add_boundary_condition(fp.analysis.DOFType.UX,Rec.left_side,0)
an_gen.add_boundary_condition(fp.analysis.DOFType.UY,Rec.left_side,0)

forc_node = mesh.find_node((80,25))
an_gen.add_force(fp.analysis.DOFType.UY,forc_node,-10)
anal = an_gen.build()


anal.set_interpolation(fp.analysis.InterpolationScheme.SIMP)
anal.update_interpolation(fp.analysis.InterpolationParameters.X_MIN,1e-3)
anal.update_interpolation(fp.analysis.InterpolationParameters.P_EXPONENT,3)

el_center = mesh.centers
rho = np.ones(mesh.number_of_elements)*1e-3
for iel in range(mesh.number_of_elements):
    if (el_center[iel,1] < 40. and el_center[iel,1] > 10.) or el_center[iel,0] < 10.:
        rho[iel] = 1

anal.update_pseudo_density(rho)

solver_dir = fp.solver.StaticSolver(anal, fp.solver.SolverType.Direct)

print(f"Elements = {mesh.number_of_elements: ,}")
print(f"DOFs     = {anal.num_free_dofs: ,}")

ti = time()
res = solver_dir.solve()
t_eigen = time()-ti
print(f"Direct time: {t_eigen} s")


print(res.compliance())
sens = res.compliance_sensitivity()
sens = np.log10(sens)

ti = time()
# res.plot_2D_grid(fp.ResultData.UY,internal_pts=100,show_edges=True, show_nodes=True)
mesh.plot_mesh2D(sens,show_edges=False,colormap='viridis')
t_plot = time()-ti
print(f"Plot time: {t_plot} s")

plt.pause(10)
plt.show()
anal.destroy()