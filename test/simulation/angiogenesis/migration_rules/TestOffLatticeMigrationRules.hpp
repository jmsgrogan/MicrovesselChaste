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

#ifndef TESTOFFLATTICEMIGRATIONRULES_HPP
#define TESTOFFLATTICEMIGRATIONRULES_HPP

#include <cxxtest/TestSuite.h>
#include "OffLatticeMigrationRule.hpp"
#include "OffLatticeSproutingRule.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "FunctionMap.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "Part.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "AngiogenesisSolver.hpp"
#include "VesselSegment.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "FlowSolver.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestOffLatticeMigrationRules : public AbstractCellBasedTestSuite
{

public:

    void Test2dMigration() throw(Exception)
    {
        BaseUnits::Instance()->SetReferenceLengthScale(1.e-6*unit::metres);
        BaseUnits::Instance()->SetReferenceTimeScale(3600.0*unit::seconds);

        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestOffLatticeMigrationRules/2d"));

        // Set up the grid
        std::shared_ptr<Part<2> > p_domain = Part<2>::Create();
        p_domain->AddRectangle(1000*1.e-6*unit::metres, 1000*1.e-6*unit::metres, DimensionalChastePoint<2>());

        std::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        QLength spacing(40.0*unit::microns); //um
        p_grid->SetSpacing(spacing);

        c_vector<double, 3> dimensions;
        dimensions[0] = 25; // num x
        dimensions[1] = 25; // num_y
        dimensions[2] = 1; // num_z
        p_grid->SetDimensions(dimensions);

        // Prescribe a linearly increasing vegf field using a function map
        std::shared_ptr<FunctionMap<2> > p_funciton_map = FunctionMap<2>::Create();
        p_funciton_map->SetGrid(p_grid);
        std::vector<QConcentration > vegf_field = std::vector<QConcentration >(dimensions[0] * dimensions[1] * dimensions[2], 0.0*unit::mole_per_metre_cubed);
        for (unsigned idx = 0; idx < dimensions[0] * dimensions[1] * dimensions[2]; idx++)
        {
            vegf_field[idx] = 0.3*p_grid->GetPoint(idx).GetLocation(spacing)[0] / (double(dimensions[0]))*1.e-9*unit::mole_per_metre_cubed;
        }

        p_grid->Write(p_handler);
        p_funciton_map->SetFileHandler(p_handler);
        p_funciton_map->SetFileName("Function");
        p_funciton_map->UpdateSolution(vegf_field);
        p_funciton_map->Write();

        //Set up the limbal vessel
        VesselNetworkGenerator<2> generator;
        QLength length = spacing * double(dimensions[1] - 1); // full domain in y direction
        unsigned divisions = dimensions[1] - 2; // divide the vessel to coincide with grid
        unsigned alignment_axis = 1; // pointing y direction
        std::shared_ptr<VesselNetwork<2> > p_network = generator.GenerateSingleVessel(length,
                                                                                        DimensionalChastePoint<2>(2.0, 0.0, 0.0, spacing),
                                                                                            divisions, alignment_axis);

        std::shared_ptr<OffLatticeMigrationRule<2> > p_migration_rule = OffLatticeMigrationRule<2>::Create();
        p_migration_rule->SetDiscreteContinuumSolver(p_funciton_map);
        p_migration_rule->SetNetwork(p_network);


        std::shared_ptr<OffLatticeSproutingRule<2> > p_sprouting_rule = OffLatticeSproutingRule<2>::Create();
        p_sprouting_rule->SetDiscreteContinuumSolver(p_funciton_map);
        p_sprouting_rule->SetVesselNetwork(p_network);
        p_sprouting_rule->SetSproutingProbability(5.e-03 /(60.0*unit::seconds));

        AngiogenesisSolver<2> angiogenesis_solver;
        angiogenesis_solver.SetVesselNetwork(p_network);
        angiogenesis_solver.SetMigrationRule(p_migration_rule);
        angiogenesis_solver.SetSproutingRule(p_sprouting_rule);
        angiogenesis_solver.SetOutputFileHandler(p_handler);
        angiogenesis_solver.SetBoundingDomain(p_domain);

        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(12.0, 24);
        angiogenesis_solver.Run(true);
    }

    void Test3dMigration() throw(Exception)
    {
        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestOffLatticeMigrationRules/3d"));

        // Set up the grid
        std::shared_ptr<Part<3> > p_domain = Part<3>::Create();
        p_domain->AddCuboid(1000*1.e-6*unit::metres, 1000*1.e-6*unit::metres, 200*1.e-6*unit::metres, DimensionalChastePoint<3>());

        std::shared_ptr<RegularGrid<3> > p_grid = RegularGrid<3>::Create();
        QLength spacing(40.0*unit::microns); //um
        p_grid->SetSpacing(spacing);

        c_vector<double, 3> dimensions;
        dimensions[0] = 25; // num x
        dimensions[1] = 25; // num_y
        dimensions[2] = 3; // num_z
        p_grid->SetDimensions(dimensions);

        // Prescribe a linearly increasing vegf field using a function map
        std::shared_ptr<FunctionMap<3> > p_funciton_map = FunctionMap<3>::Create();
        p_funciton_map->SetGrid(p_grid);
        std::vector<QConcentration > vegf_field = std::vector<QConcentration >(dimensions[0] * dimensions[1] * dimensions[2], 0.0*unit::mole_per_metre_cubed);
        for (unsigned idx = 0; idx < dimensions[0] * dimensions[1] * dimensions[2]; idx++)
        {
            vegf_field[idx] = 0.3*p_grid->GetPoint(idx).GetLocation(spacing)[0] / (double(dimensions[0]))*1.e-9*unit::mole_per_metre_cubed;
        }

        p_grid->Write(p_handler);
        p_funciton_map->SetFileHandler(p_handler);
        p_funciton_map->SetFileName("Function");
        p_funciton_map->UpdateSolution(vegf_field);
        p_funciton_map->Write();

        //Set up the limbal vessel
        VesselNetworkGenerator<3> generator;
        QLength length = spacing * double(dimensions[1] - 3); // full domain in y direction
        unsigned divisions = dimensions[1] - 2; // divide the vessel to coincide with grid
        unsigned alignment_axis = 1; // pointing y direction
        std::shared_ptr<VesselNetwork<3> > p_network = generator.GenerateSingleVessel(length,
                                                                                        DimensionalChastePoint<3>(2.0, 2.0, 0.5, spacing),
                                                                                            divisions, alignment_axis);

        std::shared_ptr<OffLatticeMigrationRule<3> > p_migration_rule = OffLatticeMigrationRule<3>::Create();
        p_migration_rule->SetDiscreteContinuumSolver(p_funciton_map);
        p_migration_rule->SetNetwork(p_network);

        std::shared_ptr<OffLatticeSproutingRule<3> > p_sprouting_rule = OffLatticeSproutingRule<3>::Create();
        p_sprouting_rule->SetDiscreteContinuumSolver(p_funciton_map);
        p_sprouting_rule->SetVesselNetwork(p_network);
        p_sprouting_rule->SetSproutingProbability(5.e-03 /(60.0*unit::seconds));

        AngiogenesisSolver<3> angiogenesis_solver;
        angiogenesis_solver.SetVesselNetwork(p_network);
        angiogenesis_solver.SetMigrationRule(p_migration_rule);
        angiogenesis_solver.SetSproutingRule(p_sprouting_rule);
        angiogenesis_solver.SetOutputFileHandler(p_handler);
        //angiogenesis_solver.SetBoundingDomain(p_domain);

        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(12.0, 24.0);
        angiogenesis_solver.Run(true);
    }
};

#endif /*TESTOFFLATTICEMIGRATIONRULES_HPP*/
