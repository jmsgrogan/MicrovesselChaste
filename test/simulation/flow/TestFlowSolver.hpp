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

#ifndef TESTFLOWSOLVER_HPP_
#define TESTFLOWSOLVER_HPP_

#include <cxxtest/TestSuite.h>
#include "VesselImpedanceCalculator.hpp"
#include "FileFinder.hpp"
#include "OutputFileHandler.hpp"
#include "SmartPointers.hpp"
#include "VesselNetworkGenerator.hpp"
#include "FlowSolver.hpp"
#include "FileFinder.hpp"
#include "VesselNetworkReader.hpp"
#include "VesselImpedanceCalculator.hpp"
#include "VesselNetworkPropertyManager.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestFlowSolver : public CxxTest::TestSuite
{
    typedef std::shared_ptr<VesselNode<2> > NodePtr2;
    typedef std::shared_ptr<VesselNode<3> > NodePtr3;
    typedef std::shared_ptr<VesselSegment<2> > SegmentPtr2;
    typedef std::shared_ptr<VesselSegment<3> > SegmentPtr3;
    typedef std::shared_ptr<Vessel<2> > VesselPtr2;
    typedef std::shared_ptr<Vessel<3> > VesselPtr3;

public:

    void TestFlowThroughSingleSegment() throw (Exception)
    {
        // Make some nodes
        std::shared_ptr<VesselNode<3> > pn1 = VesselNode<3>::Create(0, 0, 0);
        std::shared_ptr<VesselNode<3> > pn2 = VesselNode<3>::Create(5, 0, 0);

        SegmentPtr3 p_segment(VesselSegment<3>::Create(pn1, pn2));
        VesselPtr3 p_vessel(Vessel<3>::Create(p_segment));

        // Generate the network
        std::shared_ptr<VesselNetwork<3> > p_vascular_network = VesselNetwork<3>::Create();
        p_vascular_network->AddVessel(p_vessel);

        double impedance = 1.e12;
        p_segment->GetFlowProperties()->SetImpedance(impedance*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<3>::SetSegmentProperties(p_vascular_network, p_segment);

        p_vessel->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
        p_vessel->GetStartNode()->GetFlowProperties()->SetPressure(3393.0*unit::pascals);

        p_vessel->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
        p_vessel->GetEndNode()->GetFlowProperties()->SetPressure(1000.5*unit::pascals);

        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_vascular_network);
        solver.SetUp();
        solver.Solve();

        TS_ASSERT_DELTA(p_vessel->GetStartNode()->GetFlowProperties()->GetPressure()/unit::pascals, 3393, 1e-6);
        TS_ASSERT_DELTA(p_vessel->GetEndNode()->GetFlowProperties()->GetPressure()/unit::pascals, 1000.5, 1e-6);

        TS_ASSERT_DELTA(p_vessel->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - 1000.5) / impedance, 1e-6);
        TS_ASSERT_DELTA(p_segment->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - 1000.5) / impedance, 1e-6);

        p_segment->GetFlowProperties()->SetImpedance(-1.0*unit::pascal_second_per_metre_cubed);
        TS_ASSERT_THROWS_THIS(solver.Update(), "Impedance should be a positive number.");
    }

    void TestFlowThroughSingleSegmentVelocityBc() throw (Exception)
    {
        std::shared_ptr<VesselNode<3> > pn1 = VesselNode<3>::Create(0, 0, 0);
        std::shared_ptr<VesselNode<3> > pn2 = VesselNode<3>::Create(5, 0, 0);

        SegmentPtr3 p_segment(VesselSegment<3>::Create(pn1, pn2));
        VesselPtr3 p_vessel(Vessel<3>::Create(p_segment));

        // Generate the network
        std::shared_ptr<VesselNetwork<3> > p_vascular_network = VesselNetwork<3>::Create();
        p_vascular_network->AddVessel(p_vessel);

        double impedance = 1.e12;
        p_segment->GetFlowProperties()->SetImpedance(impedance*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<3>::SetSegmentProperties(p_vascular_network, p_segment);

        p_vessel->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
        p_vessel->GetStartNode()->GetFlowProperties()->SetUseVelocityBoundaryCondition(true);

        p_vessel->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
        p_vessel->GetEndNode()->GetFlowProperties()->SetPressure(0.0*unit::pascals);
        p_vessel->GetFlowProperties()->SetFlowRate(1.e-9*unit::metre_cubed_per_second);

        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_vascular_network);
        solver.SetUp();
        solver.Solve();

        TS_ASSERT_DELTA(p_vessel->GetStartNode()->GetFlowProperties()->GetPressure()/unit::pascals, 1000.0, 1e-6);
        TS_ASSERT_DELTA(p_vessel->GetEndNode()->GetFlowProperties()->GetPressure()/unit::pascals, 0.0, 1e-6);
        TS_ASSERT_DELTA(p_vessel->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, 1.e-9, 1e-6);
    }

    void TestFlowThroughBifurcationVelocityBc() throw (Exception)
    {
        std::shared_ptr<VesselNode<3> > pn1 = VesselNode<3>::Create(0, 0, 0);
        std::shared_ptr<VesselNode<3> > pn2 = VesselNode<3>::Create(5, 0, 0);
        std::shared_ptr<VesselNode<3> > pn3 = VesselNode<3>::Create(10, 5, 0);
        std::shared_ptr<VesselNode<3> > pn4 = VesselNode<3>::Create(10, -5, 0);
        SegmentPtr3 p_segment1(VesselSegment<3>::Create(pn1, pn2));
        SegmentPtr3 p_segment2(VesselSegment<3>::Create(pn2, pn3));
        SegmentPtr3 p_segment3(VesselSegment<3>::Create(pn2, pn4));
        VesselPtr3 p_vessel1(Vessel<3>::Create(p_segment1));
        VesselPtr3 p_vessel2(Vessel<3>::Create(p_segment2));
        VesselPtr3 p_vessel3(Vessel<3>::Create(p_segment3));

        // Generate the network
        std::shared_ptr<VesselNetwork<3> > p_vascular_network = VesselNetwork<3>::Create();
        p_vascular_network->AddVessel(p_vessel1);
        p_vascular_network->AddVessel(p_vessel2);
        p_vascular_network->AddVessel(p_vessel3);

        double impedance = 1.e12;
        p_segment1->GetFlowProperties()->SetImpedance(impedance*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<3>::SetSegmentProperties(p_vascular_network, p_segment1);

        p_vessel1->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
        p_vessel1->GetStartNode()->GetFlowProperties()->SetUseVelocityBoundaryCondition(true);

        p_vessel2->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
        p_vessel2->GetEndNode()->GetFlowProperties()->SetPressure(0.0*unit::pascals);
        p_vessel3->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
        p_vessel3->GetEndNode()->GetFlowProperties()->SetPressure(0.0*unit::pascals);
        p_vessel1->GetFlowProperties()->SetFlowRate(1.e-9*unit::metre_cubed_per_second);

        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_vascular_network);
        solver.SetUp();
        solver.Solve();

        TS_ASSERT_DELTA(p_vessel1->GetStartNode()->GetFlowProperties()->GetPressure()/unit::pascals, (3.0/2.0)*impedance*1.e-9, 1e-6);
        TS_ASSERT_DELTA(p_vessel1->GetEndNode()->GetFlowProperties()->GetPressure()/unit::pascals, (1.0/2.0)*impedance*1.e-9, 1e-6);
        TS_ASSERT_DELTA(p_vessel1->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, 1.e-9, 1e-6);
        TS_ASSERT_DELTA(p_vessel2->GetEndNode()->GetFlowProperties()->GetPressure()/unit::pascals, 0.0, 1e-6);
        TS_ASSERT_DELTA(p_vessel3->GetEndNode()->GetFlowProperties()->GetPressure()/unit::pascals, 0.0, 1e-6);
    }

    void TestFlowThroughSingleVesselWithMultipleSegments() throw (Exception)
    {
        // Make some nodes
        std::vector<NodePtr3> nodes;
        nodes.push_back(NodePtr3(VesselNode<3>::Create(1.0, 0, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(2.0, 0, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(3.0, 0, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(4.0, 0, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(5.0, 0, 0)));
        SegmentPtr3 p_segment1(VesselSegment<3>::Create(nodes[0], nodes[1]));
        SegmentPtr3 p_segment2(VesselSegment<3>::Create(nodes[1], nodes[2]));
        SegmentPtr3 p_segment3(VesselSegment<3>::Create(nodes[2], nodes[3]));
        SegmentPtr3 p_segment4(VesselSegment<3>::Create(nodes[3], nodes[4]));
        std::vector<SegmentPtr3> segments;
        segments.push_back(p_segment1);
        segments.push_back(p_segment2);
        segments.push_back(p_segment3);
        segments.push_back(p_segment4);

        VesselPtr3 p_vessel(Vessel<3>::Create(segments));

        // Generate the network
        std::shared_ptr<VesselNetwork<3> > p_vascular_network(new VesselNetwork<3>());
        p_vascular_network->AddVessel(p_vessel);
        double impedance = 1.e14;
        p_segment1->GetFlowProperties()->SetImpedance(1.e14*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<3>::SetSegmentProperties(p_vascular_network, p_segment1);

        p_vessel->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
        p_vessel->GetStartNode()->GetFlowProperties()->SetPressure(3393.0*unit::pascals);

        p_vessel->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
        p_vessel->GetEndNode()->GetFlowProperties()->SetPressure(1000.5* unit::pascals);

        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_vascular_network);
        solver.SetUp();
        solver.Solve();

        for (unsigned i = 0; i < nodes.size(); i++)
        {
            TS_ASSERT_DELTA(nodes[i]->GetFlowProperties()->GetPressure()/unit::pascals,
                            3393 - (3393 - 1000.5) * i / (nodes.size() - 1), 1e-6);
        }

        TS_ASSERT_DELTA(p_vessel->GetStartNode()->GetFlowProperties()->GetPressure()/unit::pascals, 3393, 1e-6);
        TS_ASSERT_DELTA(p_vessel->GetEndNode()->GetFlowProperties()->GetPressure()/unit::pascals, 1000.5, 1e-6);

        TS_ASSERT_DELTA(p_vessel->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - 1000.5) / (segments.size() * impedance), 1e-6);

        for (unsigned i = 0; i < segments.size(); i++)
        {
            TS_ASSERT_DELTA(segments[i]->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second,
                            (3393 - 1000.5) / (segments.size() * impedance), 1e-6);
        }

    }

    void TestFlowThroughMultipleVessels() throw (Exception)
    {
        // Make some nodes
        std::vector<NodePtr3> nodes;
        nodes.push_back(NodePtr3(VesselNode<3>::Create(1.0, 0, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(2.0, 0, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(3.0, 0, 0)));

        SegmentPtr3 p_segment1(VesselSegment<3>::Create(nodes[0], nodes[1]));
        SegmentPtr3 p_segment2(VesselSegment<3>::Create(nodes[1], nodes[2]));

        VesselPtr3 p_vessel1(Vessel<3>::Create(p_segment1));
        VesselPtr3 p_vessel2(Vessel<3>::Create(p_segment2));

        // Generate the network
        std::shared_ptr<VesselNetwork<3> > p_vascular_network(new VesselNetwork<3>());

        p_vascular_network->AddVessel(p_vessel1);
        p_vascular_network->AddVessel(p_vessel2);

        double impedance = 1.e14;
        p_segment1->GetFlowProperties()->SetImpedance(impedance*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<3>::SetSegmentProperties(p_vascular_network, p_segment1);

        nodes[0]->GetFlowProperties()->SetIsInputNode(true);
        nodes[0]->GetFlowProperties()->SetPressure(3393.0*unit::pascals);
        nodes[2]->GetFlowProperties()->SetIsOutputNode(true);
        nodes[2]->GetFlowProperties()->SetPressure(1000.5*unit::pascals);

        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_vascular_network);
        solver.SetUp();
        solver.Solve();

        TS_ASSERT_DELTA(p_vessel1->GetStartNode()->GetFlowProperties()->GetPressure()/unit::pascals, 3393, 1e-6);
        TS_ASSERT_DELTA(p_vessel2->GetEndNode()->GetFlowProperties()->GetPressure()/unit::pascals, 1000.5, 1e-6);
        TS_ASSERT_DELTA(nodes[1]->GetFlowProperties()->GetPressure()/unit::pascals, (3393 + 1000.5) / 2.0, 1e-6);
        TS_ASSERT_DELTA(p_vessel1->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - 1000.5) / (2.0 * impedance), 1e-6);

    }

    void TestFlowThroughBifurcation() throw (Exception)
    {

        EXIT_IF_PARALLEL;    // Need a larger network to run in parallel

        // Make some nodes
        std::vector<NodePtr3> nodes;
        nodes.push_back(NodePtr3(VesselNode<3>::Create(0.0, 0, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(0.0, 1, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(1.0, 0.5, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(1.0, 1, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(2.0, 1, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(3.0, 1, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(4.0, 1, 0)));

        SegmentPtr3 p_segment1(VesselSegment<3>::Create(nodes[0], nodes[2]));
        SegmentPtr3 p_segment2(VesselSegment<3>::Create(nodes[1], nodes[2]));
        SegmentPtr3 p_segment3(VesselSegment<3>::Create(nodes[3], nodes[2]));
        SegmentPtr3 p_segment4(VesselSegment<3>::Create(nodes[4], nodes[3]));
        SegmentPtr3 p_segment5(VesselSegment<3>::Create(nodes[5], nodes[4]));
        SegmentPtr3 p_segment6(VesselSegment<3>::Create(nodes[6], nodes[5]));

        VesselPtr3 p_vessel1(Vessel<3>::Create(p_segment1));
        VesselPtr3 p_vessel2(Vessel<3>::Create(p_segment2));
        VesselPtr3 p_vessel3(Vessel<3>::Create(p_segment3));
        VesselPtr3 p_vessel4(Vessel<3>::Create(p_segment4));
        VesselPtr3 p_vessel5(Vessel<3>::Create(p_segment5));
        VesselPtr3 p_vessel6(Vessel<3>::Create(p_segment6));

        std::vector<VesselPtr3> vessels;
        vessels.push_back(p_vessel1); // lower input vessel
        vessels.push_back(p_vessel2); // upper input vessel
        vessels.push_back(p_vessel3);
        vessels.push_back(p_vessel4);
        vessels.push_back(p_vessel5);
        vessels.push_back(p_vessel6);

        // Generate the network
        std::shared_ptr<VesselNetwork<3> > p_vascular_network(new VesselNetwork<3>());

        p_vascular_network->AddVessels(vessels);

        double impedance = 1.e14;
        p_segment1->GetFlowProperties()->SetImpedance(impedance*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<3>::SetSegmentProperties(p_vascular_network, p_segment1);

        nodes[0]->GetFlowProperties()->SetIsInputNode(true);
        nodes[0]->GetFlowProperties()->SetPressure(3393.0*unit::pascals);

        nodes[1]->GetFlowProperties()->SetIsInputNode(true);
        nodes[1]->GetFlowProperties()->SetPressure(3393.0*unit::pascals);

        nodes[6]->GetFlowProperties()->SetIsOutputNode(true);
        nodes[6]->GetFlowProperties()->SetPressure(1000.5*unit::pascals);

        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_vascular_network);
        solver.SetUp();
        solver.Solve();

        TS_ASSERT_DELTA(nodes[0]->GetFlowProperties()->GetPressure()/unit::pascals, 3393, 1e-6);
        TS_ASSERT_DELTA(nodes[1]->GetFlowProperties()->GetPressure()/unit::pascals, 3393, 1e-6);
        TS_ASSERT_DELTA(nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals, (2.0 * 3393.0 / 10.0 + 1000.5 / 40.0) / (1.0 / 40.0 + 2.0 / 10.0), 1e-6);
        TS_ASSERT_DELTA(nodes[6]->GetFlowProperties()->GetPressure()/unit::pascals, 1000.5, 1e-6);
        TS_ASSERT_DELTA(vessels[0]->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals)/ impedance, 1e-6);
        TS_ASSERT_DELTA(vessels[1]->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals) / impedance, 1e-6);
        TS_ASSERT_DELTA(vessels[5]->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, -(nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals - 1000.5) / (4.0 * impedance), 1e-6);

        TS_ASSERT_DELTA(p_segment1->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals) / impedance, 1e-6);
        TS_ASSERT_DELTA(p_segment2->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second,(3393 - nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals) / impedance, 1e-6);
        TS_ASSERT_DELTA(p_segment6->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second,-(nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals - 1000.5)/ (4.0 * impedance), 1e-6);

        double kirchoff_residual = (vessels[0]->GetFlowProperties()->GetFlowRate() +
                vessels[1]->GetFlowProperties()->GetFlowRate() +
                vessels[5]->GetFlowProperties()->GetFlowRate())/unit::metre_cubed_per_second;

        TS_ASSERT_DELTA(kirchoff_residual, 0, 1e-6);

    }

    void TestFlowThroughBifurcationHavingSwappedNodeLabels() throw (Exception)
    {
        EXIT_IF_PARALLEL;    // Need a larger network to run in parallel

        std::vector<NodePtr3> nodes;
        nodes.push_back(NodePtr3(VesselNode<3>::Create(0.0, 0, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(0.0, 1, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(1.0, 0.5, 0)));
        nodes.push_back(NodePtr3(VesselNode<3>::Create(1.0, 1, 0)));

        SegmentPtr3 p_segment1(VesselSegment<3>::Create(nodes[0], nodes[2]));
        SegmentPtr3 p_segment2(VesselSegment<3>::Create(nodes[1], nodes[2]));
        SegmentPtr3 p_segment3(VesselSegment<3>::Create(nodes[2], nodes[3]));

        VesselPtr3 p_vessel1(Vessel<3>::Create(p_segment1));
        VesselPtr3 p_vessel2(Vessel<3>::Create(p_segment2));
        VesselPtr3 p_vessel3(Vessel<3>::Create(p_segment3));

        std::vector<VesselPtr3> vessels;
        vessels.push_back(p_vessel1); // lower input vessel
        vessels.push_back(p_vessel2); // upper input vessel
        vessels.push_back(p_vessel3); // output vessel

        // Generate the network
        std::shared_ptr<VesselNetwork<3> > p_vascular_network(new VesselNetwork<3>());

        p_vascular_network->AddVessels(vessels);

        double impedance = 1.e14;
        p_segment1->GetFlowProperties()->SetImpedance(impedance*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<3>::SetSegmentProperties(p_vascular_network, p_segment1);

        nodes[0]->GetFlowProperties()->SetIsInputNode(true);
        nodes[0]->GetFlowProperties()->SetPressure(3393.0*unit::pascals);

        nodes[1]->GetFlowProperties()->SetIsInputNode(true);
        nodes[1]->GetFlowProperties()->SetPressure(3393.0*unit::pascals);

        nodes[3]->GetFlowProperties()->SetIsOutputNode(true);
        nodes[3]->GetFlowProperties()->SetPressure(1000.5*unit::pascals);

        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_vascular_network);
        solver.SetUp();
        solver.Solve();
        TS_ASSERT_DELTA(nodes[0]->GetFlowProperties()->GetPressure()/unit::pascals, 3393, 1e-6);
        TS_ASSERT_DELTA(nodes[1]->GetFlowProperties()->GetPressure()/unit::pascals, 3393, 1e-6);
        TS_ASSERT_DELTA(nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals, (2 * 3393 + 1000.5) / 3, 1e-6);
        TS_ASSERT_DELTA(nodes[3]->GetFlowProperties()->GetPressure()/unit::pascals, 1000.5, 1e-6);

        TS_ASSERT_DELTA(vessels[0]->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals)/ impedance,
                        1e-6);
        TS_ASSERT_DELTA(vessels[1]->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (3393 - nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals) / impedance,
                        1e-6);
        TS_ASSERT_DELTA(vessels[2]->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, (nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals - 1000.5) / impedance,
                        1e-6);

        TS_ASSERT_DELTA(p_segment1->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second,
                        (3393 - nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals)/ impedance, 1e-6);
        TS_ASSERT_DELTA(p_segment2->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second,
                        (3393 - nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals)/ impedance, 1e-6);
        TS_ASSERT_DELTA(p_segment3->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second,
                        (nodes[2]->GetFlowProperties()->GetPressure()/unit::pascals - 1000.5) / impedance, 1e-6);

        double kirchoff_residual = (vessels[0]->GetFlowProperties()->GetFlowRate() +
                vessels[1]->GetFlowProperties()->GetFlowRate() -
                vessels[2]->GetFlowProperties()->GetFlowRate())/unit::metre_cubed_per_second;

        TS_ASSERT_DELTA(kirchoff_residual, 0, 1e-6);

    }

    void TestFlowThroughHexagonalNetwork() throw (Exception)
    {
        // Specify the network dimensions
        QLength vessel_length = 80.0*1_um;

        // Generate the network
        VesselNetworkGenerator<2> vascular_network_generator;
        std::shared_ptr<VesselNetwork<2> > vascular_network = vascular_network_generator.GenerateHexagonalNetwork(
                1000_um, 1000_um, vessel_length);

        // Make some nodes
        std::vector<NodePtr2> nodes;
        nodes.push_back(NodePtr2(VesselNode<2>::Create(0.0, 0)));
        nodes.push_back(NodePtr2(VesselNode<2>::Create(0.0, 1)));
        SegmentPtr2 p_segment1(VesselSegment<2>::Create(nodes[0], nodes[1]));

        double impedance = 0.001;
        p_segment1->GetFlowProperties()->SetImpedance(impedance*unit::pascal_second_per_metre_cubed);
        VesselNetworkPropertyManager<2>::SetSegmentProperties(vascular_network, p_segment1);

        std::pair<Vertex<2>, Vertex<2> > network_extents = VesselNetworkGeometryCalculator<2>::GetExtents(vascular_network);
        double y_middle = (network_extents.first.GetLocation(1_um)[1]) / 2.0;

        std::vector<std::shared_ptr<Vessel<2> > >::iterator vessel_iterator;
        std::vector<std::shared_ptr<Vessel<2> > > vessels = vascular_network->GetVessels();
        for (vessel_iterator = vessels.begin(); vessel_iterator != vessels.end(); vessel_iterator++)
        {
            if ((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                if ((*vessel_iterator)->GetStartNode()->rGetLocation().GetLocation(1_um)[1] > y_middle)
                {
                    (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsInputNode(true);
                    (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(3393.0 * unit::pascals);
                }
            }
            if ((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
            {
                if ((*vessel_iterator)->GetEndNode()->rGetLocation().GetLocation(1_um)[1] > y_middle)
                {
                    (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsInputNode(true);
                    (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(3393.0 * unit::pascals);
                }
            }
            if ((*vessel_iterator)->GetStartNode()->GetNumberOfSegments() == 1)
            {
                if ((*vessel_iterator)->GetStartNode()->rGetLocation().GetLocation(1_um)[1] <= y_middle)
                {
                    (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetIsOutputNode(true);
                    (*vessel_iterator)->GetStartNode()->GetFlowProperties()->SetPressure(1993.0 * unit::pascals);
                }
            }
            if ((*vessel_iterator)->GetEndNode()->GetNumberOfSegments() == 1)
            {
                if ((*vessel_iterator)->GetEndNode()->rGetLocation().GetLocation(1_um)[1] <= y_middle)
                {
                    (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetIsOutputNode(true);
                    (*vessel_iterator)->GetEndNode()->GetFlowProperties()->SetPressure(1993.0 * unit::pascals);
                }
            }
        }

        FlowSolver<2> solver;
        solver.SetVesselNetwork(vascular_network);
        solver.SetUp();
        solver.Solve();

        // Write the network to file
        OutputFileHandler output_file_handler("TestFlowSolver", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("HexagonalVesselNetwork.vtp");
        vascular_network->Write(output_filename);
    }

    // Slow, move to long running test pack
    void DontTestFlowThroughRetinalNetwork() throw (Exception)
    {
        // Specify the network dimensions
        FileFinder fileFinder("projects/Microvessel/test/data/retinal.vtp", RelativeTo::ChasteSourceRoot);
        TS_ASSERT(fileFinder.Exists());
        TS_ASSERT(fileFinder.IsFile());

        std::shared_ptr<VesselNetworkReader<3> > p_network_reader = VesselNetworkReader<3>::Create();
        p_network_reader->SetFileName(fileFinder.GetAbsolutePath());
        p_network_reader->SetRadiusArrayName("Distance");
        std::shared_ptr<VesselNetwork<3> > p_network = p_network_reader->Read();

        p_network->GetVessel(0)->GetSegment(0)->GetFlowProperties()->SetViscosity(1.e-3 * unit::poiseuille);
        VesselNetworkPropertyManager<3>::SetSegmentProperties(p_network, p_network->GetVessel(0)->GetSegment(0));

        VesselImpedanceCalculator<3> impedance_calculator;
        impedance_calculator.SetVesselNetwork(p_network);
        impedance_calculator.Calculate();

        for(unsigned idx=0; idx<p_network->GetNodes().size(); idx++)
        {
            if(p_network->GetNodes()[idx]->rGetLocation().GetLocation(1_um)[1]<1.e-6 && p_network->GetNodes()[idx]->GetNumberOfSegments()==1)
            {
                p_network->GetNodes()[idx]->GetFlowProperties()->SetIsInputNode(true);
                p_network->GetNodes()[idx]->GetFlowProperties()->SetPressure(5000.0 * unit::pascals);
            }
            else if(p_network->GetNodes()[idx]->rGetLocation().GetLocation(1_um)[1]>1850.0  && p_network->GetNodes()[idx]->GetNumberOfSegments()==1)
            {
                p_network->GetNodes()[idx]->GetFlowProperties()->SetIsOutputNode(true);
                p_network->GetNodes()[idx]->GetFlowProperties()->SetPressure(3000.0 * unit::pascals);
            }
        }

        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetUp();
        solver.Solve();

        // Write the network to file
        OutputFileHandler output_file_handler("TestFlowSolver", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("RetinalVesselNetwork.vtp");
        p_network->Write(output_filename);
    }

    void TestLoop() throw(Exception)
    {
        EXIT_IF_PARALLEL;    // Need a larger network to run in parallel

        // Make a network
        std::vector<std::shared_ptr<VesselNode<3> > > bottom_nodes;
        for(unsigned idx=0; idx<5; idx++)
        {
            bottom_nodes.push_back(VesselNode<3>::Create(double(idx)*10, 10.0, 0.0));
        }
        bottom_nodes[0]->GetFlowProperties()->SetIsInputNode(true);
        bottom_nodes[0]->GetFlowProperties()->SetPressure(3000.0*unit::pascals);
        bottom_nodes[4]->GetFlowProperties()->SetIsOutputNode(true);
        bottom_nodes[4]->GetFlowProperties()->SetPressure(1000.0*unit::pascals);

        std::vector<std::shared_ptr<VesselNode<3> > > top_nodes;
        for(unsigned idx=1; idx<3; idx+=1)
        {
            top_nodes.push_back(VesselNode<3>::Create(double(idx)*10, 20.0, 0.0));
        }

        std::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(bottom_nodes[0], bottom_nodes[1]);
        std::shared_ptr<Vessel<3> > p_vessel2 = Vessel<3>::Create(bottom_nodes[1], bottom_nodes[2]);
        std::shared_ptr<Vessel<3> > p_vessel3 = Vessel<3>::Create(bottom_nodes[2], bottom_nodes[3]);
        std::shared_ptr<Vessel<3> > p_vessel7 = Vessel<3>::Create(bottom_nodes[3], bottom_nodes[4]);
        std::shared_ptr<Vessel<3> > p_vessel4 = Vessel<3>::Create(bottom_nodes[1], top_nodes[0]);
        std::shared_ptr<Vessel<3> > p_vessel5 = Vessel<3>::Create(bottom_nodes[2], top_nodes[1]);
        std::shared_ptr<Vessel<3> > p_vessel6 = Vessel<3>::Create(top_nodes[0], top_nodes[1]);

        std::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel1);
        p_network->AddVessel(p_vessel2);
        p_network->AddVessel(p_vessel3);
        p_network->AddVessel(p_vessel4);
        p_network->AddVessel(p_vessel5);
        p_network->AddVessel(p_vessel6);
        p_network->AddVessel(p_vessel7);
        VesselNetworkPropertyManager<3>::SetSegmentRadii(p_network, 10.0 * 1_um);
        std::vector<std::shared_ptr<VesselSegment<3> > > segments = p_network->GetVesselSegments();
        for(unsigned idx=0; idx<segments.size(); idx++)
        {
            segments[idx]->GetFlowProperties()->SetViscosity(1.e-3*unit::poiseuille);
        }

        // Grow the vessel
        VesselImpedanceCalculator<3> impedance_calculator;
        impedance_calculator.SetVesselNetwork(p_network);
        impedance_calculator.Calculate();
        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetUp();
        solver.Solve();

        // Write the network to file
        OutputFileHandler output_file_handler("TestFlowSolver", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("LoopFlow.vtp");
        p_network->Write(output_filename);
    }

    void TestSproutingWithFlow() throw(Exception)
    {
        EXIT_IF_PARALLEL;    // Need a larger network to run in parallel

        // Make a network
        std::vector<std::shared_ptr<VesselNode<3> > > bottom_nodes;
        for(unsigned idx=0; idx<6; idx++)
        {
            bottom_nodes.push_back(VesselNode<3>::Create(double(idx)*10, 10.0, 0.0));
        }
        bottom_nodes[0]->GetFlowProperties()->SetIsInputNode(true);
        bottom_nodes[0]->GetFlowProperties()->SetPressure(3000.0 * unit::pascals);
        bottom_nodes[5]->GetFlowProperties()->SetIsOutputNode(true);
        bottom_nodes[5]->GetFlowProperties()->SetPressure(1000.0 * unit::pascals);

        std::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(bottom_nodes);
        std::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel1);
        VesselNetworkPropertyManager<3>::SetSegmentRadii(p_network, 10.0 * 1_um);

        for(unsigned idx=1; idx<4; idx+=1)
        {
            Vertex<3> loc1 = Vertex<3>(double(idx)*10, 10.0, 0.0, 1_um);
            Vertex<3> loc2 = Vertex<3>(double(idx)*10, 20.0, 0.0, 1_um);
            p_network->FormSprout(VesselNode<3>::Create(loc1), loc2);
        }

        Vertex<3> loc1 = Vertex<3>(10, 20.0, 0.0, 1_um);
        Vertex<3> loc2 = Vertex<3>(20, 20.0, 0.0, 1_um);
        p_network->FormSprout(VesselNode<3>::Create(loc1), loc2);
        p_network->MergeCoincidentNodes();
        p_network->UpdateSegments();
        p_network->UpdateNodes();
        p_network->UpdateVesselNodes();
        std::vector<std::shared_ptr<VesselSegment<3> > > segments = p_network->GetVesselSegments();
        for(unsigned idx=0; idx<segments.size(); idx++)
        {
            segments[idx]->GetFlowProperties()->SetViscosity(1.e-3 * unit::poiseuille);
        }

        // Grow the vessel
        VesselImpedanceCalculator<3> impedance_calculator;
        impedance_calculator.SetVesselNetwork(p_network);
        impedance_calculator.Calculate();
        FlowSolver<3> solver;
        solver.SetVesselNetwork(p_network);
        solver.SetUp();
        solver.Solve();

        // Write the network to file
        OutputFileHandler output_file_handler("TestFlowSolver", false);
        std::string output_filename = output_file_handler.GetOutputDirectoryFullPath().append("SproutingFlow.vtp");
        p_network->Write(output_filename);
    }

};

#endif /*TESTFLOWSOLVER_HPP_*/
