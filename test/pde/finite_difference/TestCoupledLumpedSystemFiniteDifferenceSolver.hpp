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

#ifndef TESTCOUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_
#define TESTCOUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"
#include "CoupledLumpedSystemFiniteDifferenceSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestCoupledLumpedSystemFiniteDifferenceSolver : public AbstractCellBasedWithTimingsTestSuite
{

    /**
     * Diffusion only in 1D with Robin Boundary on the left
     * From Crank Diffusion text
     */
    double SolveRobinBoundary(double x, double t, double D, double k, double c_0)
    {
        double h = k/D;
        double L = std::sqrt(t*D);
        double erfc1 = erfc(x/(2.0*L));
        double erfc2 = erfc(x/(2.0*L)+h*L);
        double expterm = std::exp(h*x + h*h*D*t);
        double c = c_0*(erfc1-expterm*erfc2);
        return c;
    }
public:

    void TestPlaneSlowRelease() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.0*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.0*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(1.0*unit::seconds);

        // Set up the mesh
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(10.0*unit::metres, 100.0*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 5.0*unit::metres);

        // Choose the PDE
        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2> > p_pde = CoupledVegfPelletDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(1.0 * unit::metre_squared_per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);

        units::quantity<unit::concentration> initial_vegf_concentration(1.0*unit::mole_per_metre_cubed);
        p_pde->SetCurrentVegfInPellet(initial_vegf_concentration);
        p_pde->SetCorneaPelletPermeability(1.0*unit::metre_per_second);
        p_pde->SetPelletFreeDecayRate(0.0*unit::per_second);
        p_pde->SetPelletBindingConstant(1.0);
        p_pde->SetPelletSurfaceArea(1.0*unit::metres_squared);
        p_pde->SetPelletVolume(1.0*unit::metres*1.0*unit::metres*1.0*unit::metres);

        CoupledLumpedSystemFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestCoupledLumpedSystemFiniteDifferenceSolver/PlaneSlowRelease"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.SetTargetTimeIncrement(0.1);
        solver.SetUseCoupling(false);
        solver.SetStartTime(0.0);
        solver.SetEndTime(100.0);
        solver.SetWriteIntermediateSolutions(true, 100);
        solver.Solve();

        // Test the intermediate solutions
        vtkSmartPointer<vtkPoints> p_sample_points = vtkSmartPointer<vtkPoints>::New();
        for(unsigned idx=0; idx<11; idx++)
        {
            p_sample_points->InsertNextPoint(5.0, 100.0-idx*10.0, 0.0);
        }

        std::vector<std::pair<std::vector<double>, double> > intermediate_solutions = solver.rGetIntermediateSolutions();
        double diff_nondim = vegf_diffusivity*(1.0*unit::seconds)/(1.0*unit::metres*1.0*unit::metres);
        for(unsigned idx=0; idx<intermediate_solutions.size();idx++)
        {
            double time = intermediate_solutions[idx].second;
            if(time>10.0)
            {
                solver.UpdateSolution(intermediate_solutions[idx].first);
                std::vector<units::quantity<unit::concentration> > solution = solver.GetConcentrations(p_sample_points);
                for(unsigned jdx=0; jdx<11; jdx++)
                {
                    units::quantity<unit::length> x = double(jdx)*10.0*unit::metres;
                    double x_nondim = x/(1.0*unit::metres);
                    double c_analytical_nondim = SolveRobinBoundary(x_nondim, time, diff_nondim, 1.0, 1.0);
                    double c_numerical_nondim = solution[jdx]/initial_vegf_concentration;
                    TS_ASSERT_DELTA(c_analytical_nondim, c_numerical_nondim, 2.e-2)
                }
            }
        }
    }

    void TestPlane() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.e-6*unit::metres);
        BaseUnits::Instance()->SetReferenceConcentrationScale(1.e-6*unit::mole_per_metre_cubed);
        BaseUnits::Instance()->SetReferenceTimeScale(3600.0*unit::seconds);

        // Set up the mesh
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(2000e-6*unit::metres, 1000e-6*unit::metres, DimensionalChastePoint<2>(0.0, 0.0, 0.0));

        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        p_grid->GenerateFromPart(p_domain, 100.0e-6*unit::metres);

        // Choose the PDE
        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2> > p_pde = CoupledVegfPelletDiffusionReactionPde<2>::Create();
        units::quantity<unit::diffusivity> vegf_diffusivity(6.94e-11 * unit::metre_squared_per_second);
        units::quantity<unit::rate> vegf_decay_rate((-0.8/3600.0) * unit::per_second);
        p_pde->SetIsotropicDiffusionConstant(vegf_diffusivity);
        p_pde->SetContinuumLinearInUTerm(vegf_decay_rate);

        units::quantity<unit::concentration> initial_vegf_concentration(3.93e-4*unit::mole_per_metre_cubed);
        p_pde->SetInitialVegfInPellet(initial_vegf_concentration);

        CoupledLumpedSystemFiniteDifferenceSolver<2> solver;
        solver.SetGrid(p_grid);
        solver.SetPde(p_pde);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestCoupledLumpedSystemFiniteDifferenceSolver/Plane"));
        solver.SetFileHandler(p_output_file_handler);
        solver.SetWriteSolution(true);
        solver.SetTargetTimeIncrement(0.0005);
        solver.SetStartTime(0.0);
        solver.SetEndTime(0.5);
        solver.SetWriteIntermediateSolutions(true, 20);
        solver.Solve();
    }
};

#endif /*TESTCOUPLEDLUMPEDSYSTEMFINITEDIFFERENCESOLVER_HPP_*/
