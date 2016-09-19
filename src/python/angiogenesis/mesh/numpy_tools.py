import numpy as np

def GetMeshGrid2d(grid):
    xv = np.linspace(0,(grid.GetExtents()[0]-1)*grid.GetSpacing(),grid.GetExtents()[0])
    yv = np.linspace(0,(grid.GetExtents()[1]-1)*grid.GetSpacing(),grid.GetExtents()[1])
    X,Y = np.meshgrid(xv,yv)
    return [X,Y]

def GetMeshGridSolution(solution, grid):
    return np.reshape(solution, (grid.GetExtents()[2], grid.GetExtents()[1], -1))

def GetPlaneSolution(solution, grid, z_fraction):
    mg_solution = GetMeshGridSolution(solution, grid)
    return mg_solution[int(grid.GetExtents()[2] * z_fraction)]

def GetLineSolution(solution, grid, y_fraction, z_fraction):
    mg_solution = GetMeshGridSolution(solution, grid)
    return mg_solution[int(grid.GetExtents()[2]*z_fraction), int(grid.GetExtents()[1]*y_fraction)]

def GetXPoints(grid):
    return np.linspace(0.0, (grid.GetExtents()[0]-1)*grid.GetSpacing(), grid.GetExtents()[0])

def AddContourPlot(fig, ax, grid, solution):
    meshgrid = GetMeshGrid2d(grid)
    cs = ax.contourf(meshgrid[0], meshgrid[1], solution)
    fig.colorbar(cs, ax=ax, format="%.2f")