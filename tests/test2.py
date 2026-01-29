import finelpy as fp
import numpy as np

from time import perf_counter

from matplotlib import pyplot as plt


# Creating Geometry
L = 100
n = 10

p1 = (0,0,0)
p2 = (L,0,0)

Lin = fp.Line(p1,p2)

# Creating a new material
steel = fp.create_material("Steel")
steel.add_property(fp.MaterialProperties.RHO, 7850.0)
steel.add_property(fp.MaterialProperties.YOUNGS_MOD, 210e9)
steel.add_property(fp.MaterialProperties.POISSON, 0.3)

mesh_gen = fp.LineMesh(Lin, fp.ElementType.BAR2)
mesh_gen.set_element_material(steel)
# mesh_gen.set_element_section()
mesh_gen.create_from_element_num(n)
mesh_gen.set_integration_points(2)
mesh = mesh_gen.generate_mesh()

print(mesh.elements)
