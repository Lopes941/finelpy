import finelpy as fp
import numpy as np
import scipy as sp


from time import perf_counter as time

from matplotlib import pyplot as plt


# Creating Geometry
Lx = 80
Ly = 50
Lz = 50
nx = 80//4
ny = 50//4
nz = 50//4

O = (0,0,0)

Hex = fp.geometry.Hexahedron((Lx,Ly,Lz),O)

# Creating a new material
properties = {
    fp.material.MaterialProperties.YOUNGS_MOD: 210e9,
    fp.material.MaterialProperties.RHO: 7850,
    fp.material.MaterialProperties.POISSON: 0.3
}
steel = fp.material.create_material("Steel").add_property(properties)


el = fp.element.create_element(
        fp.element.ShapeType.HEX8, 
        fp.element.ModelType.SOLID_STRUCTURAL,
        fp.element.ConstitutiveType.SOLID_LINEAR_ELASTIC, 
        steel)

mesh_gen = fp.mesh.HexMesh(Hex, el)
mesh_gen.set_grid(nx,ny,nz)
mesh = mesh_gen.build()

an_gen = fp.analysis.AnalysisBuilder(mesh)
an_gen.add_boundary_condition(fp.analysis.DOFType.UX,Hex.left_face,0)
an_gen.add_boundary_condition(fp.analysis.DOFType.UY,Hex.left_face,0)
an_gen.add_boundary_condition(fp.analysis.DOFType.UZ,Hex.left_face,0)

forc_node = mesh.find_node((80,25,25))
an_gen.add_force(fp.analysis.DOFType.UY,forc_node,-10)
analysis = an_gen.build()


analysis.set_interpolation(fp.analysis.InterpolationScheme.SIMP)
analysis.update_interpolation(fp.analysis.InterpolationParameters.X_MIN,1e-3)
analysis.update_interpolation(fp.analysis.InterpolationParameters.P_EXPONENT,3)

el_center = mesh.centers
rho = np.ones(mesh.number_of_elements)*1e-3
for iel in range(mesh.number_of_elements):
    if (el_center[iel,1] < 40. and el_center[iel,1] > 10.) or el_center[iel,0] < 10.:
        rho[iel] = 1

analysis.update_pseudo_density(rho)

solver_dir = fp.solver.StaticSolver(analysis, fp.solver.SolverType.Direct)

print(f"Elements = {mesh.number_of_elements: ,}")
print(f"DOFs     = {analysis.num_free_dofs: ,}")

ti = time()
res = solver_dir.solve()
t_eigen = time()-ti
print(f"Direct time: {t_eigen} s")

r = (Lx,20,20)
print(f"displacement at {r} = {res.get_value(fp.results.ResultData.UY,r):0.2e}")

p, val = res.get_min(fp.results.ResultData.UY,internal_pts=1)
print(f"Minimum displacement at {p}, with value of {val:0.2e}")

sens = res.compliance_sensitivity()
sens = np.log10(sens)

mesh.plot_mesh3D(rho,show_edges=True)
mesh.plot_mesh3D(sens,show_edges=True)

analysis.destroy()
plt.show()