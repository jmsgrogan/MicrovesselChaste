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

#ifndef TESTFUNCTIONMAP_HPP_
#define TESTFUNCTIONMAP_HPP_

#include <cxxtest/TestSuite.h>
#include <vector>
#include <string>
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "SmartPointers.hpp"
#include "Part.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkGenerator.hpp"
#include "OutputFileHandler.hpp"
#include "RegularGrid.hpp"
#include "DiscreteSource.hpp"
#include "DimensionalChastePoint.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "FunctionMap.hpp"
#include "SimpleLinearEllipticFiniteElementSolver.hpp"
#include "VtkMeshWriter.hpp"
#include "GridCalculator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestFunctionMap : public CxxTest::TestSuite
{

public:

    void TestGridFunction() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.e-6*unit::metres);
        units::quantity<unit::length> length(100.0*unit::microns);

        // Set up the grid
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(length, length, DimensionalChastePoint<2>());

        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        units::quantity<unit::length> grid_spacing(5.0*unit::microns);
        p_grid->GenerateFromPart(p_domain, grid_spacing);

        // Set up a function map
        FunctionMap<2> solver;
        solver.SetGrid(p_grid);

        // Set up a quadratic function
        std::vector<double> solution;
        for(unsigned idx=0; idx<p_grid->GetNumberOfLocations(); idx++)
        {
            double x_loc = p_grid->GetLocation(idx).GetLocation(1.e-6*unit::metres)[0];
            double value = (100.0-x_loc)*(100.0-x_loc)/(100.0*100.0);
            solution.push_back(value);
        }

        solver.UpdateSolution(solution);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestFunctionMap/TestGridFunction", true));
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();
    }

    void TestMeshFunction() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.e-6*unit::metres);
        units::quantity<unit::length> length(100.0*unit::microns);

        // Set up the grid
        boost::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(length, length, DimensionalChastePoint<2>());

        boost::shared_ptr<DiscreteContinuumMeshGenerator<2> > p_mesh_generator = DiscreteContinuumMeshGenerator<2>::Create();
        p_mesh_generator->SetDomain(p_domain);
        p_mesh_generator->SetMaxElementArea(units::pow<3>(0.01*length));
        p_mesh_generator->Update();

        // Set up a function map
        FunctionMap<2> solver;
        solver.SetGrid(p_mesh_generator->GetMesh());

        // Set the source values at each point on the grid
        std::vector<double> solution;
        for(unsigned idx=0; idx<p_mesh_generator->GetMesh()->GetNumberOfLocations(); idx++)
        {
            double x_loc = p_mesh_generator->GetMesh()->GetLocation(idx).GetLocation(1.e-6*unit::metres)[0];
            double value = (100.0-x_loc)*(100.0-x_loc)/(100.0*100.0);
            solution.push_back(value);
        }
        solver.UpdateElementSolution(solution);
        MAKE_PTR_ARGS(OutputFileHandler, p_output_file_handler, ("TestFunctionMap/TestMeshFunction", false));
        solver.SetFileHandler(p_output_file_handler);
        solver.Write();
    }
};

#endif /*TESTFUNCTIONMAP_HPP_*/
