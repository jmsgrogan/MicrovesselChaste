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

#ifndef TESTSPROUTINGRULES_HPP
#define TESTSPROUTINGRULES_HPP

#include <cxxtest/TestSuite.h>
#include "AbstractCellBasedWithTimingsTestSuite.hpp"
#include "Vessel.hpp"
#include "VesselNetwork.hpp"
#include "VesselNode.hpp"
#include "FunctionMap.hpp"
#include "LatticeBasedSproutingRule.hpp"
#include "Owen2011SproutingRule.hpp"
#include "GridCalculator.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestSproutingRules : public AbstractCellBasedWithTimingsTestSuite
{

public:

    void TestLatticeBasedSproutingRuleSimpleNetwork() throw(Exception)
    {
        // Make a network
        std::vector<boost::shared_ptr<VesselNode<3> > > bottom_nodes;
        unsigned num_nodes = 100;
        double spacing = 1.0;
        for(unsigned idx=0; idx<num_nodes-1; idx++)
        {
            bottom_nodes.push_back(VesselNode<3>::Create(double(idx)*spacing, 0.0, 0.0));
        }

        boost::shared_ptr<Vessel<3> > p_vessel = Vessel<3>::Create(bottom_nodes);
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel);

        // Set up a sprouting rule
        boost::shared_ptr<LatticeBasedSproutingRule<3> > p_sprouting_rule = LatticeBasedSproutingRule<3>::Create();
        p_sprouting_rule->SetSproutingProbability(0.2*(1.0/unit::seconds));
        p_sprouting_rule->SetVesselNetwork(p_network);

        // Test that we get some, but not all, sprouts
        RandomNumberGenerator::Instance()->Reseed(522525);
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes = p_network->GetNodes();
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);

        for(unsigned idx=0; idx<5; idx++)
        {
            unsigned num_sprouts = p_sprouting_rule->GetSprouts(nodes).size();
            unsigned num_nodes = bottom_nodes.size();
            TS_ASSERT(num_sprouts>0)
            TS_ASSERT(num_sprouts<num_nodes)
        }
    }

    void TestOwen2011SproutingRuleSimpleNetwork() throw(Exception)
    {
        // Set up the grid
        boost::shared_ptr<RegularGrid<2> > p_grid = RegularGrid<2>::Create();
        double spacing = 40.0; //um
        p_grid->SetSpacing(spacing * 1.e-6 * unit::metres);
        c_vector<double, 3> dimensions;
        dimensions[0] = 101; // num x
        dimensions[1] = 11; // num_y
        dimensions[2] = 1; // num_z
        p_grid->SetDimensions(dimensions);

        boost::shared_ptr<GridCalculator<2> > p_grid_calc = GridCalculator<2>::Create();
        p_grid_calc->SetGrid(p_grid);

        // Make a network
        std::vector<boost::shared_ptr<VesselNode<2> > > bottom_nodes;
        unsigned num_nodes = 100;
        for(unsigned idx=0; idx<num_nodes-1; idx++)
        {
            bottom_nodes.push_back(VesselNode<2>::Create(double(idx)*spacing +spacing, 5.0 * spacing));
        }

        boost::shared_ptr<Vessel<2> > p_vessel = Vessel<2>::Create(bottom_nodes);
        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();
        p_network->AddVessel(p_vessel);
        p_grid_calc->SetVesselNetwork(p_network);

        // Set up a vegf field, 0.15 nM
        boost::shared_ptr<FunctionMap<2> > p_funciton_map = FunctionMap<2>::Create();
        p_funciton_map->SetGrid(p_grid);
        std::vector<units::quantity<unit::concentration> > vegf_field =
                std::vector<units::quantity<unit::concentration> >(dimensions[0]*dimensions[1], 0.15*unit::mole_per_metre_cubed);
        p_funciton_map->UpdateSolution(vegf_field);

        // Set up a sprouting rule
        boost::shared_ptr<Owen2011SproutingRule<2> > p_sprouting_rule = Owen2011SproutingRule<2>::Create();
        p_sprouting_rule->SetSproutingProbability(0.2*(1.0/unit::seconds));
        p_sprouting_rule->SetGridCalculator(p_grid_calc);
        p_sprouting_rule->SetVesselNetwork(p_network);
        p_sprouting_rule->SetDiscreteContinuumSolver(p_funciton_map);

        // Test that we get some, but not all, sprouts
        RandomNumberGenerator::Instance()->Reseed(522525);
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes = p_network->GetNodes();
        SimulationTime::Instance()->SetEndTimeAndNumberOfTimeSteps(1.0, 1);

        for(unsigned idx=0; idx<5; idx++)
        {
            unsigned num_sprouts = p_sprouting_rule->GetSprouts(nodes).size();
            unsigned num_nodes = bottom_nodes.size();
            TS_ASSERT(num_sprouts>0)
            TS_ASSERT(num_sprouts<num_nodes)
        }
    }
};

#endif
