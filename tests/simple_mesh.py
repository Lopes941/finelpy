import finelpy as fn
from time import perf_counter, sleep	

Lx = 10
Ly = 10
nx = 500
ny = 500

M = fn.Mesh()

M.add_rectangle(Lx,Ly,nx,ny)

M.plot()

print(M)


for i in range(10):
    print(f'Iteration {i}')
    M1 = fn.Mesh()

    M1.add_rectangle(Lx,Ly,nx,ny)

#print(f'Total time: {tf:0.2f} s \n Individual time: {tf/N:0.2f} s \n Number of elements: {M.num_elements:,}')

