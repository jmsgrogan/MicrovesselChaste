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

#ifndef TESTSIMPLENONLINEARELLIPTICFINITEDIFFERENCESOLVER_HPP_
#define TESTSIMPLENONLINEARELLIPTICFINITEDIFFERENCESOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestNonLinearSimpleNonLinearEllipticFiniteDifferenceSolver : public AbstractCellBasedWithTimingsTestSuite
{

private:

    /**
     * Approximate analytical solution to 1-D RD equation with MM like sink. Right boundary is fixed.
     * Left boundary has a symmetry condition.
     * http://file.scirp.org/pdf/NS_2013090214253262.pdf
     * Note: This solution seems to break down for high consumption rates. Just use it as a simple model check.
     */
    double SolveMichaelisMenten1d(double x, double gamma, double k)
    {
        double m = std::sqrt(gamma/(1.0+k));
        double cosh_m = std::cosh(m);
        double cosh_2m = std::cosh(2.0*m);
        double cosh2_m = cosh_m*cosh_m;
        double tanh_m = std::tanh(m);
        double cosh_mx = std::cosh(m*x);
        double cosh_2mx = std::cosh(2.0*m*x);
        double sinh_mx = std::sinh(m*x);

        double term1 = cosh_mx/cosh_m;
        double term2 = (cosh_2m-3.0)/(6.0*cosh2_m);
        double term3 = (k*m*m-gamma)*tanh_m/(2.0*m);
        double left = term1*(1.0+term2+term3);
        double term4 = (3.0-cosh_2mx)/(6.0*cosh2_m);
        double term5 = (gamma-k*m*m)*x*sinh_mx/(2.0*m*cosh_m);
        double c = left + term4 + term5;

        return c;
    }

public:

    void TestRectangleDomain() throw(Exception)
    {
        // Set up the grid
        BaseUnits::Instance()->SetReferenceLengthScale(1.0*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.0*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(1.0*unit::seconds);

        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(1.0*unit::metres,
                               1.0*unit::metres,
                               DimensionalChastePoint<2>(0.0, 0.0, 0.0));
        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 0.1*unit::metres);

        // Choose the PDE
        std::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<2> > p_pde =
                MichaelisMentenSteadyStateDiffusionReactionPde<2>::Create();
        QDiffusivity diffusivity(1.0* unit::metre_squared_per_second);
        QConcentrationFlowRate consumption_rate(-0.5 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetRateConstant(consumption_rate);

        QConcentration half_max_concentration(0.1 * unit::mole_per_metre_cubed);
        p_pde->SetMichaelisMentenThreshold(half_max_concentration);

        // Prescribe a value on the domain's left boundary
        std::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition = DiscreteContinuumBoundaryCondition<2>::Create();
        QConcentration boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
        p_boundary_condition->SetType(BoundaryConditionType::POINT);
        vtkSmartPointer<vtkPoints> p_boundary_points = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPoints> p_points = p_grid->GetPoints();
        for(unsigned idx=0; idx<p_points->GetNumberOfPoints(); idx++)
        {
            if(p_points->GetPoint(idx)[0]>0.99)
            {
                p_boundary_points->InsertNextPoint(p_points->GetPoint(idx));
            }
        }
        p_boundary_condition->SetPoints(p_boundary_points);

        // Set up and run the solver
        SimpleNonLinearEllipticFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_boundary_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleNonLinearEllipticFiniteDifferenceSolver/RectangleDomain", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();

        std::vector<QConcentration > solution = solver.GetConcentrations();

        for(unsigned idx=0; idx<6; idx++)
        {
            QLength x = double(idx)*0.1*unit::metres;
            double gamma = (-consumption_rate*1.0*unit::metres*1.0*unit::metres)/(boundary_concentration*diffusivity);
            double k = half_max_concentration/boundary_concentration;
            double x_nondim = x/(1.0*unit::metres);
            double c_analytical_nondim = SolveMichaelisMenten1d(x_nondim, gamma, k);
            double c_numerical_nondim = solution[idx]/(1.0* unit::mole_per_metre_cubed);
            TS_ASSERT_DELTA(c_analytical_nondim, c_numerical_nondim, 1.e-2)
        }
    }

    void TestBox() throw(Exception)
    {
        // Set up the mesh
        std::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(4.0e-6*unit::metres,
                            4.0e-6*unit::metres,
                            4.0e-6*unit::metres,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        std::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 0.5*1_um);

        // Choose the PDE
        std::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<3> > p_non_linear_pde = MichaelisMentenSteadyStateDiffusionReactionPde<3>::Create();
        QDiffusivity diffusivity(1.e-6 * unit::metre_squared_per_second);
        QConcentrationFlowRate consumption_rate(-2.e-5 * unit::mole_per_metre_cubed_per_second);
        p_non_linear_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_non_linear_pde->SetRateConstant(consumption_rate);

        QConcentration half_max_concentration(0.0625 * unit::mole_per_metre_cubed);
        p_non_linear_pde->SetMichaelisMentenThreshold(half_max_concentration);

        // Choose the Boundary conditions
        std::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_outer_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        QConcentration boundary_concentration(1.0 * unit::mole_per_metre_cubed);
        p_outer_boundary_condition->SetValue(boundary_concentration);

        SimpleNonLinearEllipticFiniteDifferenceSolver<3> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_non_linear_pde);
        solver.AddBoundaryCondition(p_outer_boundary_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleNonLinearEllipticFiniteDifferenceSolver/Box", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetFileName("output_nl_fd.vti");
        solver.SetWriteSolution(true);
        solver.Solve();
    }
};

#endif /*TESTSIMPLENONLINEARELLIPTICFINITEDIFFERENCESOLVER_HPP_*/
