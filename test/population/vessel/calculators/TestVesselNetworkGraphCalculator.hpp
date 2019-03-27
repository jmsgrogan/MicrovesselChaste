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

#ifndef TESTVESSELNETWORKGRAPHCALCULATOR_HPP_
#define TESTVESSELNETWORKGRAPHCALCULATOR_HPP_

#include <cxxtest/TestSuite.h>
#include "VesselNode.hpp"
#include "SmartPointers.hpp"
#include "ChastePoint.hpp"
#include "VesselSegment.hpp"
#include "VesselNetwork.hpp"
#include "OutputFileHandler.hpp"
#include "UblasIncludes.hpp"
#include "VesselNetworkGenerator.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkGraphCalculator.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

class TestVesselNetworkGraphCalculator : public CxxTest::TestSuite
{
public:

    void TestConnnectedMethods()
    {
        // Make some nodes
        std::vector<std::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(1.0, 2.0, 6.0));
        nodes.push_back(VesselNode<3>::Create(3.0, 4.0, 7.0));
        nodes.push_back(VesselNode<3>::Create(3.0, 4.0, 7.0));
        nodes.push_back(VesselNode<3>::Create(3.0, 4.0, 8.0));
        nodes.push_back(VesselNode<3>::Create(3.0, 4.0, 9.0));

        // Make some vessels
        std::shared_ptr<Vessel<3> > pVessel1(Vessel<3>::Create(nodes[0], nodes[1]));
        std::shared_ptr<Vessel<3> > pVessel2(Vessel<3>::Create(nodes[2], nodes[3]));
        std::shared_ptr<Vessel<3> > pVessel3(Vessel<3>::Create(nodes[3], nodes[4]));

        std::vector<std::shared_ptr<Vessel<3> > > vessels;
        vessels.push_back(pVessel2);
        vessels.push_back(pVessel3);

        // Make a network
        std::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(pVessel1);
        p_network->AddVessels(vessels);

        // Test connectivity
        TS_ASSERT_EQUALS(p_network->GetNodes().size(), 5u);
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[0]));
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[1]));
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[2]));
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[3]));
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[4]));

        // Merge coincident nodes
        p_network->MergeCoincidentNodes();
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[0]));
        TS_ASSERT(!p_network->NodeIsInNetwork(nodes[1]) != !p_network->NodeIsInNetwork(nodes[2]));
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[3]));
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[4]));
        TS_ASSERT_EQUALS(p_network->GetNodes().size(), 4u);

        std::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3>::Create(1.0 , 1.0 , 1.0);
        std::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3>::Create(5.0 , 5.0 , 1.0);
        p_network->AddVessel(Vessel<3>::Create(p_node1, p_node2));

        TS_ASSERT(p_network->NodeIsInNetwork(nodes[0]));
        // exclusive or (!A != !B)
        TS_ASSERT(!p_network->NodeIsInNetwork(nodes[1]) != !p_network->NodeIsInNetwork(nodes[2]));
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[3]));
        TS_ASSERT(p_network->NodeIsInNetwork(nodes[4]));
        TS_ASSERT(p_network->NodeIsInNetwork(p_node1));
        TS_ASSERT(p_network->NodeIsInNetwork(p_node2));

        std::shared_ptr<VesselNetworkGraphCalculator<3> > p_graph_calculator = VesselNetworkGraphCalculator<3>::Create();
        p_graph_calculator->SetVesselNetwork(p_network);

        TS_ASSERT(p_graph_calculator->IsConnected(nodes[0], nodes[4]));
        TS_ASSERT(!p_graph_calculator->IsConnected(nodes[0], p_node2));
        TS_ASSERT(p_graph_calculator->IsConnected(p_node1, p_node2));

        std::vector<std::shared_ptr<VesselNode<3> > > source_nodes;
        source_nodes.push_back(nodes[0]);
        source_nodes.push_back(p_node1);

        std::vector<std::shared_ptr<VesselNode<3> > > query_nodes;
        query_nodes.push_back(nodes[0]);
        if (p_network->NodeIsInNetwork(nodes[1]))
        {
            query_nodes.push_back(nodes[1]);
        }
        if (p_network->NodeIsInNetwork(nodes[2]))
        {
            query_nodes.push_back(nodes[2]);
        }
        query_nodes.push_back(nodes[3]);
        query_nodes.push_back(nodes[4]);

        std::vector<bool> connected = p_graph_calculator->IsConnected(source_nodes, query_nodes);
        TS_ASSERT(connected[0]);
        TS_ASSERT(connected[1]);
        TS_ASSERT(connected[2]);
        TS_ASSERT(connected[3]);

        OutputFileHandler output_file_handler("TestVesselNetworkGraphCalculator",false);
        std::string output_filename4 = output_file_handler.GetOutputDirectoryFullPath().append("ConnectedTestVesselNetwork.gv");
        p_graph_calculator->WriteConnectivity(output_filename4);
    }
};

#endif /*TESTVESSELNETWORKGRAPHCALCULATOR_HPP_*/
