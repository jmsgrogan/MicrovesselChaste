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

#ifndef TESTMICROVESSELSOLVER_HPP
#define TESTMICROVESSELSOLVER_HPP

#include <cxxtest/TestSuite.h>
#include "LatticeBasedMigrationRule.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "Part.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "AngiogenesisSolver.hpp"
#include "MicrovesselSolver.hpp"
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "SimulationTime.hpp"
#include "PetscSetupAndFinalize.hpp"

class TestMicrovesselSolver : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestSingleVesselGrowth() throw(Exception)
    {
        // Make a network
        boost::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3>::Create(0.0, 0.0, 0.0);
        boost::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3>::Create(100.0, 0.0, 0.0);
        p_node2->SetIsMigrating(true);
        boost::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(VesselSegment<3>::Create(p_node1, p_node2));
        boost::shared_ptr<VesselNetwork<3> > p_network = boost::shared_ptr<VesselNetwork<3> >(new VesselNetwork<3>());
        p_network->AddVessel(p_vessel1);
        p_network->GetVessels()[0]->GetEndNode()->SetIsMigrating(true);

        // Set up an angiogenesis solver
        boost::shared_ptr<LatticeBasedMigrationRule<3> > p_migration_rule = LatticeBasedMigrationRule<3>::Create();
        boost::shared_ptr<AngiogenesisSolver<3> > p_angiogenesis_solver = AngiogenesisSolver<3>::Create();
        p_angiogenesis_solver->SetVesselNetwork(p_network);
        p_angiogenesis_solver->SetMigrationRule(p_migration_rule);

        // Set up a vascular tumour solver
        MicrovesselSolver<3> vascular_tumour_solver;
        vascular_tumour_solver.SetVesselNetwork(p_network);

        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(10.0, 10);
        MAKE_PTR_ARGS(OutputFileHandler, p_file_handler, ("TestMicrovesselSolver/SingleVesselGrowth/"));
        vascular_tumour_solver.SetOutputFileHandler(p_file_handler);
        vascular_tumour_solver.Run();
    }
};

#endif //TESTMICROVESSELSOLVER_HPP
