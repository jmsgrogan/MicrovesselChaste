/*

Copyright (c) 2005-2017, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef TESTSIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_
#define TESTSIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "ParabolicDiffusionReactionPde.hpp"
#include "SimpleParabolicFiniteDifferenceSolver.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestSimpleParabolicFiniteDifferenceSolver : public AbstractCellBasedWithTimingsTestSuite
{

private:

    /**
     * Hill 1928 Fixed bounary on both ends, diffusion only.
     */
    double Solve1DParabolic(double x, double D, double t)
    {
        double b = 0.5;
        double c = 1.0;
        for(unsigned idx=0; idx<10; idx++)
        {
            double n = 2*idx + 1;
            c = c - (4.0/M_PI)*(1.0/n)*std::exp(-n*n*D*M_PI*M_PI*t/(4.0*b*b))*std::sin(n*M_PI*x/(2.0*b));
        }
        return c;
    }

public:

    void TestRectangleDomain() throw(Exception)
    {
        // Set up the grid
        BaseUnits::Instance()->SetReferenceLengthScale(1.0*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.0*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(1.0*unit::seconds);

        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(1.0*unit::metres,
                               1.0*unit::metres,
                               DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 0.1*unit::metres);

        // Choose the PDE
        boost::shared_ptr<ParabolicDiffusionReactionPde<2> > p_pde =
                ParabolicDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> diffusivity(1.0* unit::metre_squared_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);

        // Prescribe a value on the domain's left and right boundary
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition = DiscreteContinuumBoundaryCondition<2>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::POINT);
        vtkSmartPointer<vtkPoints> p_boundary_points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPoints> p_points = p_grid->GetPoints();
        for(unsigned idx=0; idx<p_points->GetNumberOfPoints(); idx++)
        {
            if(p_points->GetPoint(idx)[0]>0.99 or p_points->GetPoint(idx)[0]<0.01)
            {
                p_boundary_points->InsertNextPoint(p_points->GetPoint(idx));
            }
        }
        p_boundary_condition->SetPoints(p_boundary_points);

        std::vector<double> initial_condition(p_grid->GetNumberOfPoints(), 0.0);

        // Set up and run the solver
        SimpleParabolicFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_boundary_condition);
        solver.UpdateSolution(initial_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleParabolicFiniteDifferenceSolver/RectangleDomain"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.SetTargetTimeIncrement(0.0005);
        solver.SetStartTime(0.0);
        solver.SetEndTime(0.5);
        solver.SetWriteIntermediateSolutions(true, 20);
        solver.Solve();

        // Test the intermediate solutions
        std::vector<std::pair<std::vector<double>, double> > intermediate_solutions =
                solver.rGetIntermediateSolutions();
        double diff_nondim = diffusivity*(1.0*unit::seconds)/(1.0*unit::metres*1.0*unit::metres);
        for(unsigned idx=0; idx<intermediate_solutions.size();idx++)
        {
            double time = intermediate_solutions[idx].second;
            if(time>0.1) // Analytical solution won't capture sharp initial condition.
            {
                solver.UpdateSolution(intermediate_solutions[idx].first);
                std::vector<units::quantity<unit::concentration> > solution = solver.GetConcentrations();
                for(unsigned jdx=0; jdx<11; jdx++)
                {
                    units::quantity<unit::length> x = double(jdx)*0.1*unit::metres;
                    double x_nondim = x/(1.0*unit::metres);
                    double c_analytical_nondim = Solve1DParabolic(x_nondim, diff_nondim, time);
                    double c_numerical_nondim = solution[jdx]/boundary_concentration;
                    TS_ASSERT_DELTA(c_analytical_nondim, c_numerical_nondim, 1.e-2)
                }
            }
        }
    }

    void xTestBox() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-6*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(3600.0*unit::seconds);

        // Set up the mesh
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(1000e-6*unit::metres, 1000e-6*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 100.0e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<ParabolicDiffusionReactionPde<2> > p_pde = ParabolicDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        units::quantity<unit::rate> vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_pde->SetContinuumLinearInUTerm(vegf_decay_rate);

        units::quantity<unit::concentration> initial_vegf_concentration(3.93e-4*unit::mole_per_metre_cubed);
        std::vector<double> initial_condition(p_grid->GetNumberOfPoints(), double(initial_vegf_concentration/(1.e-6*unit::mole_per_metre_cubed)));

        SimpleParabolicFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.UpdateSolution(initial_condition);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleParabolicFiniteDifferenceSolver/Box"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.SetTargetTimeIncrement(0.01);
        solver.SetStartTime(0.0);
        solver.SetEndTime(1.0);
        solver.SetWriteIntermediateSolutions(true, 10);
        solver.Solve();
    }
};

#endif /*TESTSIMPLEPARABOLICFINITEDIFFERENCESOLVER_HPP_*/
