import finelpy as fp

import numpy as np
from finelpy.elements.line_elements import BarElement
from matplotlib import pyplot as plt


# Creating Geometry
L = 1
n = 2

def forc(p: np.ndarray) -> float:
    # x = p[0]
    return 4


line = fp.Line(L)

# Creating a new material
properties = {
    fp.MaterialProperties.YOUNGS_MOD: 1e9,
    fp.MaterialProperties.A: 1e-6
}
steel = fp.create_material("Steel").add_property(properties)

el = BarElement(steel,1)

mesh_gen = fp.LineMesh(line, el)
mesh_gen.create_from_element_num(n)
mesh = mesh_gen.build()


an_gen = fp.AnalysisBuilder(mesh)
an_gen.add_boundary_condition(fp.DOFType.UX,0,0)
an_gen.add_force(fp.DOFType.UX,fp.Line(0,L),forc,10)
an_gen.add_force(fp.DOFType.UX,mesh.number_of_nodes-1,10)
analysis = an_gen.build()

solver_dir = fp.StaticSolver(analysis)
res = solver_dir.solve()

int_pts = 100
## Plot displacement
ax,x = res.plot_data1D(fp.ResultData.UX,internal_pts=int_pts, show_nodes=True)

analytical_disp = 1/(steel.E*steel.A) * (-2*x**2 + 14*x)
ax.plot(x, analytical_disp, 'r--')
ax.legend(["Solução de elementos finitos", "Solução analítica"])
fig = plt.gcf()
fig.set_size_inches(7, 2)
fig.savefig("Deslocamento_exemplo_4-1")

## Plot strain
ax,x = res.plot_data1D(fp.ResultData.EPSILON_XX,internal_pts=int_pts, show_nodes=True)

analytical_strain = -0.004*x + 0.014
ax.plot(x, analytical_strain, 'r--')
ax.legend(["Solução de elementos finitos", "Solução analítica"])
fig = plt.gcf()
fig.set_size_inches(7, 2)
fig.savefig("Deformacao_exemplo_4-1")


## Plot stress
ax,x = res.plot_data1D(fp.ResultData.SIGMA_XX,internal_pts=int_pts, show_nodes=True)

analytical_strain = steel.E*(-0.004*x + 0.014)
ax.plot(x, analytical_strain, 'r--')
ax.legend(["Solução de elementos finitos", "Solução analítica"])
fig = plt.gcf()
fig.set_size_inches(7, 2)
fig.savefig("Tensao_exemplo_4-1")


# ## Plot stress
# ax,x = res.plot_data1D(fp.ResultData.SIGMA_XX,internal_pts=int_pts)

# analytical_sig = 1/(steel.A) * (-x**2/2 + L**2/2)
# #ax.plot(x, analytical_sig, 'r--', label='Analytical Solution')
# ax.legend()


# x, u_fem = res.grid_data(fp.ResultData.EPSILON_XX,internal_pts=int_pts)

# x = x[:,0]
# #u_an = 1/(steel.E*steel.A) * (-x**2/2 + L**2/2)
# u_an = 1/(steel.E*steel.A) * (-x**3/6 + x*L**2/2)

# e = np.abs(u_an-u_fem)/(u_an+1e-7) *100

# #plt.figure()
# #plt.plot(x,e,'b-',label='erro')
# #plt.xlabel('x [m]')
# #plt.ylabel('Displacement [m]')
# #plt.legend()



plt.show()



