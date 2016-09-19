''' Tests for the DiscreteContinuum solver module.
'''

import unittest
import math
import os
import numpy as np
import petsc4py, sys
import chaste.geometry
import chaste.mesh
import chaste.pde

class TestFiniteDifferenceSolver(unittest.TestCase):
          
    def test_fixed_outer_boundary(self):
        domain = chaste.geometry.Part()
        domain.AddCuboid(100.0, 100.0, 100.0)
        
        grid = chaste.mesh.RegularGrid()
        grid.GenerateFromPart(domain, 10.0)
        
        pde = chaste.pde.LinearSteadyStateDiffusionReactionPde()
        pde.SetIsotropicDiffusionConstant(0.003)
        pde.SetContinuumLinearInUTerm(-1.e-5)
        
        bc = chaste.pde.DiscreteContinuumBoundaryCondition()
        bc.SetValue(30.0)
        
#         file_handler = chaste.core.OutputFileHandler("chaste/TestFiniteDifferenceSolver")
        solver = chaste.pde.FiniteDifferenceSolver()
        solver.SetGrid(grid)
        solver.SetPde(pde)
        solver.AddBoundaryCondition(bc)
#         solver.SetFileHandler(file_handler)
#         solver.SetFileName("OuterBoundary")
#         solver.SetWriteSolution(True)
        solver.Solve()
#         print solver.GetVtkSolution()
        
    def test_fixed_left_boundary(self):
        domain = chaste.geometry.Part()
        domain.AddCuboid(100.0, 100.0, 10.0)
        
        grid = chaste.mesh.RegularGrid()
        grid.GenerateFromPart(domain, 10.0)
        
        pde = chaste.pde.LinearSteadyStateDiffusionReactionPde()
        pde.SetIsotropicDiffusionConstant(0.003)
        pde.SetContinuumLinearInUTerm(-1.e-5)
        
        for eachFacet in domain.GetFacets():
            if eachFacet.GetCentroid()[0] == 0.0:
                eachFacet.SetData("Boundary", 30.0)
        
        bc = chaste.pde.DiscreteContinuumBoundaryCondition()
        bc.SetType(chaste.pde.BoundaryConditionType.FACET)
        bc.SetSource(chaste.pde.BoundaryConditionSource.LABEL_BASED)
        bc.SetLabelName("Boundary")
        bc.SetDomain(domain)
        
        solver = chaste.pde.FiniteDifferenceSolver()
        solver.SetGrid(grid)
        solver.SetPde(pde)
        solver.AddBoundaryCondition(bc)
        solver.Solve()
        
    def test_fixed_left_right_boundary(self):
        domain = chaste.geometry.Part()
        domain.AddCuboid(100.0, 100.0, 10.0)
        
        grid = chaste.mesh.RegularGrid()
        grid.GenerateFromPart(domain, 10.0)
        
        pde = chaste.pde.LinearSteadyStateDiffusionReactionPde()
        pde.SetIsotropicDiffusionConstant(0.003)
        pde.SetContinuumLinearInUTerm(-1.e-5)
        
        for eachFacet in domain.GetFacets():
            if eachFacet.GetCentroid()[0] == 0.0:
                eachFacet.SetData("LeftBoundary", 30.0)
        
        for eachFacet in domain.GetFacets():
            if eachFacet.GetCentroid()[0] == 100.0:
                eachFacet.SetData("RightBoundary", 15.0)
        
        left_bc = chaste.pde.DiscreteContinuumBoundaryCondition()
        left_bc.SetType(chaste.pde.BoundaryConditionType.FACET)
        left_bc.SetSource(chaste.pde.BoundaryConditionSource.LABEL_BASED)
        left_bc.SetLabelName("LeftBoundary")
        left_bc.SetDomain(domain)
        
        right_bc = chaste.pde.DiscreteContinuumBoundaryCondition()
        right_bc.SetType(chaste.pde.BoundaryConditionType.FACET)
        right_bc.SetSource(chaste.pde.BoundaryConditionSource.LABEL_BASED)
        right_bc.SetLabelName("RightBoundary")
        right_bc.SetDomain(domain)
        
        solver = chaste.pde.FiniteDifferenceSolver()
        solver.SetGrid(grid)
        solver.SetPde(pde)
        solver.AddBoundaryCondition(left_bc)
        solver.AddBoundaryCondition(right_bc)
        solver.Solve()
        