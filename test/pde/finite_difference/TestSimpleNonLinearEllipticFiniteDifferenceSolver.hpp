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

#include "PetscSetupAndFinalize.hpp"

class TestNonLinearSimpleNonLinearEllipticFiniteDifferenceSolver : public AbstractCellBasedWithTimingsTestSuite
{
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
        boost::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<2> > p_pde =
                MichaelisMentenSteadyStateDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> diffusivity(1.0* unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-10.0 * unit::mole_per_metre_cubed_per_second);
        p_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_pde->SetRateConstant(consumption_rate);

        units::quantity<unit::concentration> half_max_concentration(0.1 * unit::mole_per_metre_cubed);
        p_pde->SetMichaelisMentenThreshold(half_max_concentration);

        // Prescribe a value on the domain's left boundary
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<2> > p_boundary_condition = DiscreteContinuumBoundaryCondition<2>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.0* unit::mole_per_metre_cubed);
        p_boundary_condition->SetValue(boundary_concentration);
//        p_boundary_condition->SetType(BoundaryConditionType::POINT);
//        vtkSmartPointer<vtkPoints> p_boundary_points = vtkSmartPointer<vtkPoints>::New();
//        vtkSmartPointer<vtkPoints> p_points = p_grid->GetLocations();
//        for(unsigned idx=0; idx<p_points->GetNumberOfPoints(); idx++)
//        {
//            if(p_points->GetPoint(idx)[0]==1.0)
//            {
//                p_boundary_points->InsertNextPoint(p_points->GetPoint(idx));
//            }
//        }
//        p_boundary_condition->SetPoints(p_boundary_points);

        // Set up and run the solver
        SimpleNonLinearEllipticFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_boundary_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleNonLinearEllipticFiniteDifferenceSolver/RectangleDomain", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();

        std::vector<units::quantity<unit::concentration> > solution = solver.GetConcentrations();

        // Analytical http://file.scirp.org/pdf/NS_2013090214253262.pdf
        // c = (cosh(mx)/cosh(m))*(1+(cosh(2m-3)/(6*cosh^2(m)))+(k*m^2-\gamma/(2*a))*tanh(m))+
        // (((3-cosh(2mx))/(6cosh^2(m)))+(((\gamma -km^2)xsinh(mx)/(2mcosh(m)))))

        for(unsigned idx=0; idx<6; idx++)
        {
            units::quantity<unit::length> x = double(idx)*1.0*unit::metres;
            units::quantity<unit::length> w = 5.0*unit::metres;
            units::quantity<unit::concentration> c = -consumption_rate*x*x/(2.0*diffusivity)-
                    x*-consumption_rate*w/diffusivity + boundary_concentration;
            double norm_analytical = c/(1.0* unit::mole_per_metre_cubed);
            double norm_numerical = solution[idx]/(1.0* unit::mole_per_metre_cubed);
            TS_ASSERT_DELTA(norm_analytical, norm_numerical, 1.e-6)
        }
    }

    void TestBox() throw(Exception)
    {
        // Set up the mesh
        boost::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(4.0e-6*unit::metres,
                            4.0e-6*unit::metres,
                            4.0e-6*unit::metres,
                            DimensionalChastePoint<3>(0.0, 0.0, 0.0));

        boost::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        p_grid->GenerateFromPart(p_domain, 0.5*1.e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<3> > p_non_linear_pde = MichaelisMentenSteadyStateDiffusionReactionPde<3>::Create();
        units::quantity<unit::diffusivity> diffusivity(1.e-6 * unit::metre_squared_per_second);
        units::quantity<unit::concentration_flow_rate> consumption_rate(-2.e-5 * unit::mole_per_metre_cubed_per_second);
        p_non_linear_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_non_linear_pde->SetRateConstant(consumption_rate);

        units::quantity<unit::concentration> half_max_concentration(0.0625 * unit::mole_per_metre_cubed);
        p_non_linear_pde->SetMichaelisMentenThreshold(half_max_concentration);

        // Choose the Boundary conditions
        boost::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_outer_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        units::quantity<unit::concentration> boundary_concentration(1.0 * unit::mole_per_metre_cubed);
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
