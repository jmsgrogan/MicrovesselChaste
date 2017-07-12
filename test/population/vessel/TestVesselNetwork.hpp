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

#ifndef TESTVESSELNETWORK_HPP_
#define TESTVESSELNETWORK_HPP_

#include <cxxtest/TestSuite.h>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPolyData.h>
#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkCellLocator.h>
#include <vtkLine.h>
#include "VesselNode.hpp"
#include "SmartPointers.hpp"
#include "ChastePoint.hpp"
#include "VesselSegment.hpp"
#include "VesselNetwork.hpp"
#include "OutputFileHandler.hpp"
#include "UblasIncludes.hpp"
#include "VesselNetworkGenerator.hpp"
#include "UnitCollection.hpp"
#include "Timer.hpp"
#include "VesselNetworkGeometryCalculator.hpp"
#include "VesselNetworkPropertyManager.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVesselNetwork : public CxxTest::TestSuite
{
public:

    void TestAddingVessels() throw(Exception)
    {
        // Make some nodes
        std::vector<VesselNodePtr<3> > nodes;
        for(unsigned idx=0; idx < 4; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(double(idx)*1_um));
        }

        // Make some vessels
        std::vector<std::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 1; idx++)
        {
            vessels.push_back(Vessel<3>::Create(nodes[idx], nodes[idx+1]));
        }
        std::shared_ptr<Vessel<3> > p_end_vessel = Vessel<3>::Create(nodes[2], nodes[3]);

        // Make a network
        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);
        vessel_network.AddVessel(p_end_vessel);
        TS_ASSERT_EQUALS(vessel_network.GetNodes().size(), 4u);
    }

    void TestSettingNetworkData() throw(Exception)
    {
        // Make some nodes
        std::vector<VesselNodePtr<3> > nodes;
        for(unsigned idx=0; idx < 4; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(double(idx)*1_um));
        }

        // Make some vessels
        std::vector<std::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 2; idx++)
        {
            vessels.push_back(Vessel<3>::Create(nodes[idx], nodes[idx+1]));
        }

        // Make a network
        std::shared_ptr<VesselNetwork<3> > p_vessel_network = VesselNetwork<3>::Create();
        p_vessel_network->AddVessels(vessels);
        VesselNetworkPropertyManager<3>::SetNodeRadii(p_vessel_network, 10.0e-6 * unit::metres);
        VesselNetworkPropertyManager<3>::SetSegmentRadii(p_vessel_network, 12.0e-6 * unit::metres);
    }

    void TestCopyingAndMovingNetwork() throw(Exception)
    {
        // Make some nodes
        std::vector<VesselNodePtr<3> > nodes;
        for(unsigned idx=0; idx < 4; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(double(idx)*1_um));
        }

        // Make some vessels
        std::vector<std::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 3; idx++)
        {
            vessels.push_back(Vessel<3>::Create(VesselSegment<3>::Create(nodes[idx], nodes[idx+1])));
        }

        // Make a network
        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);
        vessel_network.MergeCoincidentNodes();

        // Move the network
        QLength reference_length(1_um);
        Vertex<3> translation_vector(0.0_um, 2.0_um, 0.0_um);
        vessel_network.Translate(translation_vector);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[0]->GetSegments()[0]->GetNode(0)->rGetLocation().Convert(reference_length)[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[0]->GetSegments()[0]->GetNode(0)->rGetLocation().Convert(reference_length)[1], 2.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[0]->GetSegments()[0]->GetNode(1)->rGetLocation().Convert(reference_length)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[0]->GetSegments()[0]->GetNode(1)->rGetLocation().Convert(reference_length)[1], 2.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[1]->GetSegments()[0]->GetNode(1)->rGetLocation().Convert(reference_length)[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[1]->GetSegments()[0]->GetNode(1)->rGetLocation().Convert(reference_length)[1], 2.0, 1.e-6);

        // Copy the network
        std::vector<std::shared_ptr<Vessel<3> > > copied_vessels = vessel_network.CopyVessels();
        TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 6u);

        // Move the new vessels
        Vertex<3> translation_vector2(0.0_um, 0.0_um, 3.0_um);
        vessel_network.Translate(translation_vector2);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[3]->GetSegments()[0]->GetNode(1)->rGetLocation().Convert(reference_length)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[3]->GetSegments()[0]->GetNode(1)->rGetLocation().Convert(reference_length)[1], 2.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[3]->GetSegments()[0]->GetNode(1)->rGetLocation().Convert(reference_length)[2], 3.0, 1.e-6);
    }

    void TestRemoveVessel() throw(Exception)
    {
        // Make some nodes
        std::vector<VesselNodePtr<3> > nodes;
        nodes.push_back(VesselNode<3>::Create(0.0_um));
        nodes.push_back(VesselNode<3>::Create(20.0_um));
        nodes.push_back(VesselNode<3>::Create(30.0_um));
        nodes.push_back(VesselNode<3>::Create(50.0_um));

        // Make some vessels
        std::vector<std::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 3; idx++)
        {
            vessels.push_back(Vessel<3>::Create(VesselSegment<3>::Create(nodes[idx], nodes[idx+1])));
        }

        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);

        vessel_network.RemoveShortVessels(15.0e-6 * unit::metres, false);
        TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 2u);
    }

    void TestDivideVessel() throw(Exception)
    {
         // Make some nodes
         std::vector<VesselNodePtr<3> > nodes;
         for(unsigned idx=0; idx < 2; idx++)
         {
             nodes.push_back(VesselNode<3>::Create(2.0 * double(idx)*1_um));
         }

         // Make a network
         VesselNetwork<3> vessel_network;
         vessel_network.AddVessel(Vessel<3>::Create(nodes[0], nodes[1]));

         TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 1u);
         TS_ASSERT_EQUALS(vessel_network.GetNumberOfNodes(), 2u);

         // Do the divide
         QLength reference_length(1.0*unit::microns);
         Vertex<3> location(0.66_um);
         vessel_network.DivideVessel(vessel_network.GetVessels()[0], location);
         TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 2u);
         TS_ASSERT_EQUALS(vessel_network.GetNumberOfNodes(), 3u);
         TS_ASSERT_DELTA(vessel_network.GetVessel(0)->GetSegment(0)->GetNode(0)->rGetLocation().Convert(reference_length)[0], 0.0, 1.e-6);
         TS_ASSERT_DELTA(vessel_network.GetVessel(0)->GetSegment(0)->GetNode(1)->rGetLocation().Convert(reference_length)[0], 0.66, 1.e-6);
         TS_ASSERT_DELTA(vessel_network.GetVessel(1)->GetSegment(0)->GetNode(0)->rGetLocation().Convert(reference_length)[0], 0.66, 1.e-6);
         TS_ASSERT_DELTA(vessel_network.GetVessel(1)->GetSegment(0)->GetNode(1)->rGetLocation().Convert(reference_length)[0], 2.0, 1.e-6);
    }

    void TestDivideMultiSegmentVessel() throw(Exception)
    {
        // Make some nodes
        std::vector<VesselNodePtr<3> > nodes;
        for(unsigned idx=1; idx < 6; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(double(idx)*1_um));
        }

        // Generate the network
        VesselNetwork<3> p_network;
        p_network.AddVessel(Vessel<3>::Create(nodes));

        TS_ASSERT_EQUALS(p_network.GetNumberOfVessels(), 1u);
        TS_ASSERT_EQUALS(p_network.GetNumberOfNodes(), 5u);

        // Do the divide
        Vertex<3> location(3_um);
        p_network.DivideVessel(p_network.GetVessels()[0], location);
        TS_ASSERT_EQUALS(p_network.GetNumberOfVessels(), 2u);
        TS_ASSERT_EQUALS(p_network.GetNumberOfNodes(), 5u);

        VesselSegmentPtr<3> p_v0_seg_0 = p_network.GetVessel(0)->GetSegment(0);
        VesselSegmentPtr<3> p_v0_seg_1 = p_network.GetVessel(0)->GetSegment(1);
        VesselSegmentPtr<3> p_v1_seg_0 = p_network.GetVessel(1)->GetSegment(0);
        VesselSegmentPtr<3> p_v1_seg_1 = p_network.GetVessel(1)->GetSegment(1);

        TS_ASSERT_DELTA(p_v0_seg_0->GetNode(0)->rGetLocation().Convert(1_um)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(p_v0_seg_0->GetNode(1)->rGetLocation().Convert(1_um)[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_v0_seg_1->GetNode(0)->rGetLocation().Convert(1_um)[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_v0_seg_1->GetNode(1)->rGetLocation().Convert(1_um)[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_v1_seg_0->GetNode(0)->rGetLocation().Convert(1_um)[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_v1_seg_0->GetNode(1)->rGetLocation().Convert(1_um)[0], 4.0, 1.e-6);
        TS_ASSERT_DELTA(p_v1_seg_1->GetNode(0)->rGetLocation().Convert(1_um)[0], 4.0, 1.e-6);
        TS_ASSERT_DELTA(p_v1_seg_1->GetNode(1)->rGetLocation().Convert(1_um)[0], 5.0, 1.e-6);

        Vertex<3> location2(4.5_um);
        TS_ASSERT_THROWS_ANYTHING(p_network.DivideVessel(p_network.GetVessel(0), location2));

        // Do the divide
        p_network.DivideVessel(p_network.GetVessel(1), location2);
        TS_ASSERT_EQUALS(p_network.GetNumberOfVessels(), 3u);
        TS_ASSERT_EQUALS(p_network.GetNumberOfNodes(), 6u);
        TS_ASSERT_DELTA(p_network.GetVessel(0)->GetSegment(0)->GetNode(0)->rGetLocation().Convert(1_um)[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(0)->GetSegment(0)->GetNode(1)->rGetLocation().Convert(1_um)[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(0)->GetSegment(1)->GetNode(0)->rGetLocation().Convert(1_um)[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(0)->GetSegment(1)->GetNode(1)->rGetLocation().Convert(1_um)[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(1)->GetSegment(0)->GetNode(0)->rGetLocation().Convert(1_um)[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(1)->GetSegment(0)->GetNode(1)->rGetLocation().Convert(1_um)[0], 4.0, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(1)->GetSegment(1)->GetNode(0)->rGetLocation().Convert(1_um)[0], 4.0, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(1)->GetSegment(1)->GetNode(1)->rGetLocation().Convert(1_um)[0], 4.5, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(2)->GetSegment(0)->GetNode(0)->rGetLocation().Convert(1_um)[0], 4.5, 1.e-6);
        TS_ASSERT_DELTA(p_network.GetVessel(2)->GetSegment(0)->GetNode(1)->rGetLocation().Convert(1_um)[0], 5.0, 1.e-6);
    }

    void TestSprouting()
    {
        auto p_vessel_network = VesselNetwork<2>::Create();
        std::vector<VesselNodePtr<2> > nodes;
        nodes.push_back(VesselNode<2>::Create(0_um));
        nodes.push_back(VesselNode<2>::Create(1_um));
        nodes.push_back(VesselNode<2>::Create(2_um));
        auto p_vessel = Vessel<2>::Create(nodes);
        p_vessel_network->AddVessel(p_vessel);

        // form sprout
        Vertex<2> sprout_tip(0.0_um, 1.0_um, 0.0_um);
        VesselPtr<2> p_new_sprout = p_vessel_network->FormSprout(nodes[1], sprout_tip);

        p_vessel_network->UpdateAll(true);
        // test number of vessels and nodes in network
        TS_ASSERT_EQUALS(p_vessel_network->GetNumberOfNodes(),4u);
        TS_ASSERT_EQUALS(p_vessel_network->GetNumberOfVessels(),3u);
    }

    void TestRemoveAndDeleteVessel() throw(Exception)
    {
        // Make some nodes
        std::vector<VesselNodePtr<3> > nodes;
        nodes.push_back(VesselNode<3>::Create(0.0_um));
        nodes.push_back(VesselNode<3>::Create(20.0_um));
        nodes.push_back(VesselNode<3>::Create(30.0_um));
        nodes.push_back(VesselNode<3>::Create(50.0_um));

        // Make some vessels
        std::vector<std::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 3; idx++)
        {
            vessels.push_back(Vessel<3>::Create(VesselSegment<3>::Create(nodes[idx], nodes[idx+1])));
        }

        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);

        vessel_network.RemoveVessel(vessels[0]);
        TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 2u);

        vessel_network.UpdateNodes();
        TS_ASSERT_EQUALS(vessel_network.GetNumberOfNodes(), 3u);
    }

    void TestMergeVessel() throw(Exception)
    {
        // Make some nodes
        std::vector<VesselNodePtr<3> > nodes;
        nodes.push_back(VesselNode<3>::Create(0.0_um));
        nodes.push_back(VesselNode<3>::Create(20.0_um));
        nodes.push_back(VesselNode<3>::Create(30.0_um));
        nodes.push_back(VesselNode<3>::Create(50.0_um));

        // Make some vessels
        std::vector<std::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 3; idx++)
        {
            vessels.push_back(Vessel<3>::Create(VesselSegment<3>::Create(nodes[idx], nodes[idx+1])));
        }

        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);

        vessel_network.MergeShortVessels(15.0e-6 * unit::metres);
        TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 2u);
        TS_ASSERT_DELTA(vessels[0]->GetEndNode()->rGetLocation().Convert(1e-6 * unit::metres)[0], 20.0, 1.e-6);
        TS_ASSERT_DELTA(vessels[2]->GetStartNode()->rGetLocation().Convert(1e-6 * unit::metres)[0], 20.0, 1.e-6);
    }

    void TestMultipleSprouts() throw(Exception)
    {
        // Make a network
        std::vector<VesselNodePtr<3> > bottom_nodes;
        for(unsigned idx=0; idx<3; idx++)
        {
            bottom_nodes.push_back(VesselNode<3>::Create(double(idx)*10_um));
        }

        auto p_vessel = Vessel<3>::Create(bottom_nodes);
        auto p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel);

        // Add some sprouts
        for(unsigned idx=1; idx<2; idx++)
        {
            Vertex<3> tip_location(double(idx)*10_um, 10_um);
            p_network->FormSprout(bottom_nodes[idx], tip_location);
        }

        // make sure vessels are correctly divided
        std::vector<VesselPtr<3> > vessels = p_network->GetVessels();
        TS_ASSERT_EQUALS(vessels.size(), 3u);
        for(unsigned idx=0; idx<vessels.size(); idx++)
        {
            TS_ASSERT_DELTA(vessels[idx]->GetLength()/1_um, 10.0, 1.e-6);
        }
        OutputFileHandler output_file_handler("TestVesselNetwork",false);
        p_network->Write(output_file_handler.GetOutputDirectoryFullPath() + "/multisprout.vtp");
    }

    void TestFindNearestSegment()
    {
        std::vector<VesselNodePtr<2> > nodes;
        nodes.push_back(VesselNode<2>::Create(0.0_um));
        nodes.push_back(VesselNode<2>::Create(10.0_um));
        nodes.push_back(VesselNode<2>::Create(20.0_um));
        nodes.push_back(VesselNode<2>::Create(16.0_um, 5_um));

        // Make some vessels
        std::vector<std::shared_ptr<Vessel<2> > > vessels;
        vessels.push_back(Vessel<2>::Create(VesselSegment<2>::Create(nodes[0], nodes[1])));
        vessels.push_back(Vessel<2>::Create(VesselSegment<2>::Create(nodes[1], nodes[2])));

        auto p_vessel_network = VesselNetwork<2>::Create();
        p_vessel_network->AddVessels(vessels);

        VesselSegmentPtr<2> p_nearest_segment;
        QLength distance = VesselNetworkGeometryCalculator<2>::GetNearestSegment(p_vessel_network, nodes[3],
                p_nearest_segment, false);
        TS_ASSERT(p_nearest_segment);
        TS_ASSERT_DELTA(double(distance/(1_um)), 5.0, 1.e-6);
    }
};

#endif /*TESTVESSELNETWORK_HPP_*/
