import finelpy as fp

import numpy as np
from finelpy.element.line_elements import BarElement, BeamElement
from matplotlib import pyplot as plt


# Creating Geometry
L = 10
n = 4

def forc(p: np.ndarray) -> float:
    x = p[0]
    return -x



line = fp.Line(L)

# Creating a new material
properties = {
    fp.MaterialProperties.YOUNGS_MOD: 210e9,
    fp.MaterialProperties.IZZ: 10e-4
}
steel = fp.create_material("Steel").add_property(properties)

el = BeamElement(steel)

mesh_gen = fp.LineMesh(line, el)
mesh_gen.create_from_element_num(n)
mesh = mesh_gen.build()


an_gen = fp.AnalysisBuilder(mesh)
an_gen.add_boundary_condition(fp.DOFType.UY,0,0)
an_gen.add_boundary_condition(fp.DOFType.THETAZ,0,0)
an_gen.add_force(fp.DOFType.UY,fp.Line(0,L),forc,3)
anal = an_gen.build()



solver_dir = fp.StaticSolver(anal, fp.SolverType.Direct)
res = solver_dir.solve()

int_pts = 100

## Plot displacement
ax,x = res.plot_data1D(fp.ResultData.UY,internal_pts=int_pts)

C1 = 1 * L**2/2
C2 = 1 * L**3/6 - C1*L
C3 = 0
C4 = 0

u_an = 1/(steel.E*steel.IZZ) * (-x**5/120 + C1*x**3/6 + C2*x**2/2 + C3*x + C4)
ax.plot(x, u_an, 'r--')
ax.set_title("Displacement")
ax.set_xlabel("x")
ax.set_ylabel("v(x)")
ax.legend(['Approximate solution', 'Analytical solution'])

## Plot angle
ax,x = res.plot_data1D(fp.ResultData.THETAZ,internal_pts=int_pts)
theta_an = 1/(steel.E*steel.IZZ) * (-x**4/24 + C1*x**2/2 + C2*x + C3)
ax.plot(x, theta_an, 'r--')
ax.set_title("Angle")
ax.set_xlabel("x")
ax.set_ylabel("theta(x)")
ax.legend(['Approximate solution', 'Analytical solution'])

## Plot bending moment
ax,x = res.plot_data1D(fp.ResultData.MZ,internal_pts=int_pts)
Mz_an = (-x**3/6 + C1*x + C2)
ax.plot(x, Mz_an, 'r--')
ax.set_title("Bending Moment")
ax.set_xlabel("x")
ax.set_ylabel("Mz(x)")
ax.legend(['Approximate solution', 'Analytical solution'])


## Plot shear force
ax,x = res.plot_data1D(fp.ResultData.VY,internal_pts=int_pts)
Vy_an = (-x**2/2 + C1)
ax.plot(x, Vy_an, 'r--')
ax.set_title("Shear Force")
ax.set_xlabel("x")
ax.set_ylabel("Vy(x)")
ax.legend(['Approximate solution', 'Analytical solution'])


#x, u_fem = res.grid_data(fp.ResultData.EPSILON_XX,internal_pts=int_pts)

# x = x[:,0]
# #u_an = 1/(steel.E*steel.A) * (-x**2/2 + L**2/2)
# u_an = 1/(steel.E*steel.A) * (-x**3/6 + x*L**2/2)

# e = np.abs(u_an-u_fem)/(u_an+1e-7) *100

#plt.figure()
#plt.plot(x,e,'b-',label='erro')
#plt.xlabel('x [m]')
#plt.ylabel('Displacement [m]')
#plt.legend()



plt.show()



