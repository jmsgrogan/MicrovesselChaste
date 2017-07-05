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

#ifndef TESTSIMPLENONLINEARELLIPTICFINITEELEMENTSOLVER_HPP_
#define TESTSIMPLENONLINEARELLIPTICFINITEELEMENTSOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <boost/lexical_cast.hpp>
#include "SimpleNonLinearEllipticFiniteElementSolver.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "SmartPointers.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestSimpleNonLinearEllipticFiniteElementSolver : public AbstractCellBasedWithTimingsTestSuite
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

        DiscreteContinuumMeshGenerator<2> mesh_generator;
        mesh_generator.SetDomain(p_domain);
        mesh_generator.SetMaxElementArea(0.01*(Qpow3(1.0*unit::metres)));
        mesh_generator.Update();
        std::shared_ptr<DiscreteContinuumMesh<2> > p_mesh = mesh_generator.GetMesh();

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
        TetrahedralMesh<2,2>::BoundaryElementIterator surf_iter = p_mesh->GetBoundaryElementIteratorBegin();
        while (surf_iter != p_mesh->GetBoundaryElementIteratorEnd())
        {
            unsigned node_index = (*surf_iter)->GetNodeGlobalIndex(0);
            double x = p_mesh->GetNode(node_index)->GetPoint()[0];
            if (x>0.999)
            {
                p_boundary_points->InsertNextPoint(p_mesh->GetNode(node_index)->GetPoint()[0],
                        p_mesh->GetNode(node_index)->GetPoint()[1], 0.0);
            }
            surf_iter++;
        }
        p_boundary_condition->SetPoints(p_boundary_points);

        // Set up and run the solver
        SimpleNonLinearEllipticFiniteElementSolver<2> solver;
        solver.SetGrid(p_mesh);
        solver.SetPde(p_pde);
        solver.AddBoundaryCondition(p_boundary_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleNonLinearEllipticFiniteElementSolver/RectangleDomain", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.Solve();

        vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
        for(unsigned idx=0; idx<11; idx++)
        {
            p_sample_points->InsertNextPoint(double(idx)*0.1, 0.5, 0.0);
        }

        std::vector<QConcentration > solution = solver.GetConcentrations(p_sample_points);
        for(unsigned idx=0; idx<11; idx++)
        {
            QLength x = double(idx)*0.1*unit::metres;
            double gamma = (-consumption_rate*1.0*unit::metres*1.0*unit::metres)/(boundary_concentration*diffusivity);
            double k = half_max_concentration/boundary_concentration;
            double x_nondim = x/(1.0*unit::metres);
            double c_analytical_nondim = SolveMichaelisMenten1d(x_nondim, gamma, k);
            double c_numerical_nondim = solution[idx]/boundary_concentration;
            TS_ASSERT_DELTA(c_analytical_nondim, c_numerical_nondim, 1.e-2)
        }
    }

    void TestBox() throw(Exception)
    {
        // Set up the mesh
        std::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(100.0e-6*unit::metres, 100.0e-6*unit::metres, 100.0e-6*unit::metres, DimensionalChastePoint<3>(0.0, 0.0, 0.0));
        std::shared_ptr<DiscreteContinuumMeshGenerator<3, 3> > p_mesh_generator = DiscreteContinuumMeshGenerator<3, 3>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(2000.0*Qpow3(1.e-6*unit::metres));
        p_mesh_generator->Update();

        // Choose the PDE
        std::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<3> > p_non_linear_pde =
                MichaelisMentenSteadyStateDiffusionReactionPde<3>::Create();
        QDiffusivity diffusivity(1.0* unit::metre_squared_per_second);
        QConcentrationFlowRate consumption_rate(-0.0005 * unit::mole_per_metre_cubed_per_second);
        p_non_linear_pde->SetIsotropicDiffusionConstant(diffusivity);
        p_non_linear_pde->SetRateConstant(consumption_rate);

        // Choose the Boundary conditions
        std::shared_ptr<DiscreteContinuumBoundaryCondition<3> > p_outer_boundary_condition = DiscreteContinuumBoundaryCondition<3>::Create();
        QConcentration boundary_concentration(1.0 * unit::mole_per_metre_cubed);
        p_outer_boundary_condition->SetValue(boundary_concentration);

        SimpleNonLinearEllipticFiniteElementSolver<3> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());
        solver.SetPde(p_non_linear_pde);
        solver.AddBoundaryCondition(p_outer_boundary_condition);

        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestSimpleNonLinearEllipticFiniteElementSolver/Box", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetFileName("output_nl_t");
        solver.SetWriteSolution(true);
        //solver.SetUseSimpleNetonSolver(true);
        solver.Solve();
    }
};

#endif /*TESTSIMPLENONLINEARELLIPTICFINITEELEMENTSOLVER_HPP_*/
