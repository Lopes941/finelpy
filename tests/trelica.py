import numpy as np
import finelpy as fp
from finelpy import TrussElementExercise
import matplotlib.pyplot as plt

class Trelica(TrussElementExercise):

    def __init__(self, material):
        super().__init__(material)

    def Ke(self,ue=None):

        he = self.L

        E = self.material.E
        A = self.material.A

        Ke_loc = (E*A/he)*np.array([[1,0,-1,0],[0,0,0,0],[-1,0,1,0],[0,0,0,0]])

        R = self.R

        self.ke.value = R.T @ Ke_loc @ R

        return self.ke.value
    
    def get_strain(self, loc, ue):
        u_bar = self.R @ ue
        u_bar = u_bar[[0,2]]
        return self.B(loc, ue) @ u_bar

    def get_stress(self, loc, ue):
        return self.material.E * self.get_strain(loc, ue)
    

coord = np.array([
    [0,0,0],
    [4,0,0],
    [8,0,0],
    [12,0,0],
    [8,3,0],
    [4,3,0]
], dtype=float)

inci = np.array([
    [0,1],
    [1,2],
    [2,3],
    [3,4],
    [4,5],
    [0,5],
    [1,5],
    [2,5],
    [2,4]
], dtype=int)

properties = {
    fp.MaterialProperties.YOUNGS_MOD: 210e9,
    fp.MaterialProperties.A: 1e-5
}

mat = fp.create_material("Material").add_property(properties)

el = Trelica(mat)

mesh_gen = fp.FrameMesh(el,coord,inci)
mesh = mesh_gen.build()


an_gen = fp.AnalysisBuilder(mesh)
an_gen.add_boundary_condition(fp.DOFType.UX, mesh.find_node((0,0)),0)
an_gen.add_boundary_condition(fp.DOFType.UY,mesh.find_node((0,0)),0)
an_gen.add_boundary_condition(fp.DOFType.UY,mesh.find_node((12,0)),0)

an_gen.add_force(fp.DOFType.UX,mesh.find_node((8,3)),400)
an_gen.add_force(fp.DOFType.UY,mesh.find_node((8,0)),-1200)
analysis = an_gen.build()

solver_dir = fp.StaticSolver(analysis)
res = solver_dir.solve()


tensao = res.element_mean(fp.ResultData.SIGMA_XX,1)

fig,ax = mesh.plot_lines(values=tensao,show_colorbar=True,colormap='coolwarm')

for el,te in enumerate(tensao):
    print(f"{el}: {te/1e6:0.2f} MPa")

fig.set_size_inches(8,5)
fig.savefig("tensao_trelica.png")
plt.show()
