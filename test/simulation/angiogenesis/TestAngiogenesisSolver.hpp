/*

Copyright (c) 2005-2016, University of Oxford.
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

#ifndef TESTANGIOGENESISSOLVER_HPP
#define TESTANGIOGENESISSOLVER_HPP

#include <cxxtest/TestSuite.h>
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "RandomNumberGenerator.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "LatticeBasedMigrationRule.hpp"
#include "LatticeBasedSproutingRule.hpp"
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "AngiogenesisSolver.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestAngiogenesisSolver : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void Test2dLatticeBasedSingleVesselGrowth() throw(Exception)
    {
        RandomNumberGenerator::Instance()->Reseed(123456);

        // Set the grid to move on
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        double spacing = 20.0; //um
        p_grid->SetSpacing(spacing * 1.e-6 * unit::metres);
        std::vector<unsigned> extents(3, 1);
        extents[0] = 7; // num x
        extents[1] = 5; // num_y
        extents[2] = 1; // num_z
        p_grid->SetExtents(extents);

        // Make a vessel
        boost::shared_ptr<VesselNode<2> > p_node1 = VesselNode<2>::Create(0.0, 2.0*spacing);
        boost::shared_ptr<VesselNode<2> > p_node2 = VesselNode<2>::Create(spacing, 2.0*spacing);
        p_node2->SetIsMigrating(true);
        boost::shared_ptr<Vessel<2> > p_vessel = Vessel<2>::Create(p_node1, p_node2);
        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel);
        p_grid->SetVesselNetwork(p_network);

        // Set up the migration rule
        boost::shared_ptr<LatticeBasedMigrationRule<2> > p_migration_rule = LatticeBasedMigrationRule<2>::Create();
        p_migration_rule->SetGrid(p_grid);
        p_migration_rule->SetMovementProbability(0.1);
        p_migration_rule->SetNetwork(p_network);

        // Grow the vessel
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(30, 30);
        AngiogenesisSolver<2> angiogenesis_solver;
        angiogenesis_solver.SetVesselNetwork(p_network);
        angiogenesis_solver.SetMigrationRule(p_migration_rule);
        angiogenesis_solver.SetVesselGrid(p_grid);

        MAKE_PTR_ARGS(OutputFileHandler, p_handler, ("TestAngiogenesisSolver/Lattice/SingleVessel"));
        angiogenesis_solver.SetOutputFileHandler(p_handler);
        angiogenesis_solver.Run(true);
    }
};

#endif // TESTANGIOGENESISSOLVER_HPP
