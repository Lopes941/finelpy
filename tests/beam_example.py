import finelpy as fp

from matplotlib import pyplot as plt

# Creating Geometry
Lx = 100e-3
Ly = 10e-3
ny = 100
nx = ny*10
t = 1

O = (0,0,0)

Rec = fp.Rectangle((Lx,Ly),O)

# Creating a new material
properties = {
    fp.MaterialProperties.YOUNGS_MOD: 210e9,
    fp.MaterialProperties.RHO: 7850,
    fp.MaterialProperties.POISSON: 0.3
}

steel = fp.create_material("Steel").add_property(properties)


el = fp.create_element(
        fp.ShapeType.TRI3, 
        fp.ModelType.PLANE_STRUCTURAL,
        fp.ConstitutiveType.PLANE_STRESS, 
        steel)

mesh_gen = fp.RectangularMesh(Rec, el)
mesh_gen.set_grid(nx,ny)
mesh = mesh_gen.build()

f = 10e3
forc_node = Rec.right_side.nodes
if forc_node.size%2 == 0:
    forc_node = forc_node[forc_node.size//2:forc_node.size//2+2]
else:
    forc_node = forc_node[forc_node.size//2+1]


an_gen = fp.AnalysisBuilder(mesh)
an_gen.add_boundary_condition(fp.DOFType.UX,Rec.left_side,0)
an_gen.add_boundary_condition(fp.DOFType.UY,Rec.left_side,0)
an_gen.add_force(fp.DOFType.UY,forc_node,-f/forc_node.size)
anal = an_gen.build()

solver_dir = fp.StaticSolver(anal, fp.SolverType.Direct)

print(f"Elements = {mesh.number_of_elements: ,}")
print(f"DOFs     = {anal.num_free_dofs: ,}")

res = solver_dir.solve()


Mz = f*Lx
Izz = (t*Ly**3)/12
sigma_xx = Mz*(Ly/2)/Izz

print(f"Theoretical sigma_xx = {sigma_xx/1e6:.2f} MPa")
max_sigma_xx = res.get_max(fp.ResultData.SIGMA_XX)
print(f"Numerical   sigma_xx = {max_sigma_xx/1e6:.2f} MPa")


fig,ax = mesh.plot_mesh2D()

fig,ax = res.plot_data2D(fp.ResultData.SIGMA_XX,show_colorbar=True,gauss_points=2,exponent_factor=0.5)



plt.show()
#anal.destroy()