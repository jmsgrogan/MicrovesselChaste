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

#ifndef TESTVESSELNETWORK_HPP_
#define TESTVESSELNETWORK_HPP_

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

#include "PetscSetupAndFinalize.hpp"

class TestVesselNetwork : public CxxTest::TestSuite
{
public:

    void TestAddingVessels() throw(Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        for(unsigned idx=0; idx < 4; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(double(idx), 0.0, 0.0));
        }

        // Make some vessels
        std::vector<boost::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 1; idx++)
        {
            vessels.push_back(Vessel<3>::Create(nodes[idx], nodes[idx+1]));
        }
        boost::shared_ptr<Vessel<3> > p_end_vessel = Vessel<3>::Create(nodes[2], nodes[3]);

        // Make a network
        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);
        vessel_network.AddVessel(p_end_vessel);
        TS_ASSERT_EQUALS(vessel_network.GetNodes().size(), 4u);
    }

    void TestSettingNetworkData() throw(Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        for(unsigned idx=0; idx < 4; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(double(idx), 0.0, 0.0));
        }

        // Make some vessels
        std::vector<boost::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 2; idx++)
        {
            vessels.push_back(Vessel<3>::Create(nodes[idx], nodes[idx+1]));
        }

        // Make a network
        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);

        vessel_network.SetNodeRadii(10.0e-6 * unit::metres);
        vessel_network.SetSegmentRadii(12.0e-6 * unit::metres);
    }

    void TestCopyingAndMovingNetwork() throw(Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        for(unsigned idx=0; idx < 4; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(double(idx), 0.0, 0.0));
        }

        // Make some vessels
        std::vector<boost::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 3; idx++)
        {
            vessels.push_back(Vessel<3>::Create(VesselSegment<3>::Create(nodes[idx], nodes[idx+1])));
        }

        // Make a network
        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);
        vessel_network.MergeCoincidentNodes();

        // Move the network
        c_vector<double, 3> translation_vector = zero_vector<double>(3);
        translation_vector[1] = 2.0;
        vessel_network.Translate(translation_vector);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[0]->GetSegments()[0]->GetNode(0)->rGetLocation()[0], 0.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[0]->GetSegments()[0]->GetNode(0)->rGetLocation()[1], 2.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[0]->GetSegments()[0]->GetNode(1)->rGetLocation()[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[0]->GetSegments()[0]->GetNode(1)->rGetLocation()[1], 2.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[1]->GetSegments()[0]->GetNode(1)->rGetLocation()[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[1]->GetSegments()[0]->GetNode(1)->rGetLocation()[1], 2.0, 1.e-6);

        // Copy the network
        std::vector<boost::shared_ptr<Vessel<3> > > copied_vessels = vessel_network.CopyVessels();
        TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 6u);

        // Move the new vessels
        c_vector<double, 3> translation_vector2 = zero_vector<double>(3);
        translation_vector2[2] = 3.0;
        vessel_network.Translate(translation_vector2);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[3]->GetSegments()[0]->GetNode(1)->rGetLocation()[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[3]->GetSegments()[0]->GetNode(1)->rGetLocation()[1], 2.0, 1.e-6);
        TS_ASSERT_DELTA(vessel_network.GetVessels()[3]->GetSegments()[0]->GetNode(1)->rGetLocation()[2], 3.0, 1.e-6);
    }

    void TestRemoveVessel() throw(Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(0.0, 0.0, 0.0));
        nodes.push_back(VesselNode<3>::Create(20.0, 0.0, 0.0));
        nodes.push_back(VesselNode<3>::Create(30.0, 0.0, 0.0));
        nodes.push_back(VesselNode<3>::Create(50.0, 0.0, 0.0));

        // Make some vessels
        std::vector<boost::shared_ptr<Vessel<3> > > vessels;
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
         std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
         for(unsigned idx=0; idx < 2; idx++)
         {
             nodes.push_back(VesselNode<3>::Create(2.0 * double(idx)));
         }

         // Make a network
         VesselNetwork<3> vessel_network;
         vessel_network.AddVessel(Vessel<3>::Create(nodes[0], nodes[1]));

         TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 1u);
         TS_ASSERT_EQUALS(vessel_network.GetNumberOfNodes(), 2u);

         // Do the divide
         c_vector<double, 3> location = zero_vector<double>(3);
         location[0] = 0.66;
         vessel_network.DivideVessel(vessel_network.GetVessels()[0], location);
         TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 2u);
         TS_ASSERT_EQUALS(vessel_network.GetNumberOfNodes(), 3u);
         TS_ASSERT_DELTA(vessel_network.GetVessel(0)->GetSegment(0)->GetNode(0)->rGetLocation()[0], 0.0, 1.e-6);
         TS_ASSERT_DELTA(vessel_network.GetVessel(0)->GetSegment(0)->GetNode(1)->rGetLocation()[0], 0.66, 1.e-6);
         TS_ASSERT_DELTA(vessel_network.GetVessel(1)->GetSegment(0)->GetNode(0)->rGetLocation()[0], 0.66, 1.e-6);
         TS_ASSERT_DELTA(vessel_network.GetVessel(1)->GetSegment(0)->GetNode(1)->rGetLocation()[0], 2.0, 1.e-6);
    }

    void TestDivideMultiSegmentVessel() throw(Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        for(unsigned idx=1; idx < 6; idx++)
        {
            nodes.push_back(VesselNode<3>::Create(double(idx), 0.0, 0.0));
        }

        // Generate the network
        VesselNetwork<3> p_vascular_network;
        p_vascular_network.AddVessel(Vessel<3>::Create(nodes));

        TS_ASSERT_EQUALS(p_vascular_network.GetNumberOfVessels(), 1u);
        TS_ASSERT_EQUALS(p_vascular_network.GetNumberOfNodes(), 5u);

        // Do the divide
        c_vector<double, 3> location = zero_vector<double>(3);
        location[0] = 3.0;
        p_vascular_network.DivideVessel(p_vascular_network.GetVessels()[0], location);
        TS_ASSERT_EQUALS(p_vascular_network.GetNumberOfVessels(), 2u);
        TS_ASSERT_EQUALS(p_vascular_network.GetNumberOfNodes(), 5u);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(0)->GetSegment(0)->GetNode(0)->rGetLocation()[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(0)->GetSegment(0)->GetNode(1)->rGetLocation()[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(0)->GetSegment(1)->GetNode(0)->rGetLocation()[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(0)->GetSegment(1)->GetNode(1)->rGetLocation()[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(1)->GetSegment(0)->GetNode(0)->rGetLocation()[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(1)->GetSegment(0)->GetNode(1)->rGetLocation()[0], 4.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(1)->GetSegment(1)->GetNode(0)->rGetLocation()[0], 4.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(1)->GetSegment(1)->GetNode(1)->rGetLocation()[0], 5.0, 1.e-6);

        c_vector<double, 3> location2 = zero_vector<double>(3);
        location2[0] = 4.5;
        TS_ASSERT_THROWS_ANYTHING(p_vascular_network.DivideVessel(p_vascular_network.GetVessel(0), location2));

        // Do the divide
        p_vascular_network.DivideVessel(p_vascular_network.GetVessel(1), location2);
        TS_ASSERT_EQUALS(p_vascular_network.GetNumberOfVessels(), 3u);
        TS_ASSERT_EQUALS(p_vascular_network.GetNumberOfNodes(), 6u);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(0)->GetSegment(0)->GetNode(0)->rGetLocation()[0], 1.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(0)->GetSegment(0)->GetNode(1)->rGetLocation()[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(0)->GetSegment(1)->GetNode(0)->rGetLocation()[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(0)->GetSegment(1)->GetNode(1)->rGetLocation()[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(1)->GetSegment(0)->GetNode(0)->rGetLocation()[0], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(1)->GetSegment(0)->GetNode(1)->rGetLocation()[0], 4.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(1)->GetSegment(1)->GetNode(0)->rGetLocation()[0], 4.0, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(1)->GetSegment(1)->GetNode(1)->rGetLocation()[0], 4.5, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(2)->GetSegment(0)->GetNode(0)->rGetLocation()[0], 4.5, 1.e-6);
        TS_ASSERT_DELTA(p_vascular_network.GetVessel(2)->GetSegment(0)->GetNode(1)->rGetLocation()[0], 5.0, 1.e-6);
    }

    void TestSprouting()
    {
        boost::shared_ptr<VesselNetwork<2> > p_vessel_network = VesselNetwork<2>::Create();
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes1;
        nodes1.push_back(VesselNode<2>::Create(0));
        nodes1.push_back(VesselNode<2>::Create(1));
        nodes1.push_back(VesselNode<2>::Create(2));
        boost::shared_ptr<Vessel<2> > p_vessel1 = Vessel<2>::Create(nodes1);

        p_vessel_network->AddVessel(p_vessel1);

        // form sprout
        c_vector<double, 2> sproutBaseLocation = zero_vector<double>(2);
        sproutBaseLocation[0] = 1.0;
        c_vector<double, 2> sproutTipLocation = unit_vector<double>(2, 1);
        boost::shared_ptr<Vessel<2> > newSprout = p_vessel_network->FormSprout(sproutBaseLocation, sproutTipLocation);

        p_vessel_network->UpdateAll(true);
        // test number of vessels and nodes in network
        TS_ASSERT_EQUALS(p_vessel_network->GetNumberOfNodes(),4u);
        TS_ASSERT_EQUALS(p_vessel_network->GetNumberOfVessels(),3u);
    }

    void TestRemoveAndDeleteVessel() throw(Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(0.0));
        nodes.push_back(VesselNode<3>::Create(20.0));
        nodes.push_back(VesselNode<3>::Create(30.0));
        nodes.push_back(VesselNode<3>::Create(50.0));

        // Make some vessels
        std::vector<boost::shared_ptr<Vessel<3> > > vessels;
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
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(0.0));
        nodes.push_back(VesselNode<3>::Create(20.0));
        nodes.push_back(VesselNode<3>::Create(30.0));
        nodes.push_back(VesselNode<3>::Create(50.0));

        // Make some vessels
        std::vector<boost::shared_ptr<Vessel<3> > > vessels;
        for(unsigned idx=0; idx < 3; idx++)
        {
            vessels.push_back(Vessel<3>::Create(VesselSegment<3>::Create(nodes[idx], nodes[idx+1])));
        }

        VesselNetwork<3> vessel_network;
        vessel_network.AddVessels(vessels);

        vessel_network.MergeShortVessels(15.0e-6 * unit::metres);
        TS_ASSERT_EQUALS(vessel_network.GetNumberOfVessels(), 2u);
        TS_ASSERT_DELTA(vessels[0]->GetEndNode()->rGetLocation()[0], 20.0, 1.e-6);
        TS_ASSERT_DELTA(vessels[2]->GetStartNode()->rGetLocation()[0], 20.0, 1.e-6);
    }

    void TestMultipleSprouts() throw(Exception)
    {
        // Make a network
        std::vector<boost::shared_ptr<VesselNode<3> > > bottom_nodes;
        for(unsigned idx=0; idx<3; idx++)
        {
            bottom_nodes.push_back(VesselNode<3>::Create(double(idx)*10, 0.0, 0.0));
        }

        boost::shared_ptr<Vessel<3> > p_vessel1 = Vessel<3>::Create(bottom_nodes);
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(p_vessel1);

        // Add some sprouts
        for(unsigned idx=1; idx<2; idx++)
        {
            c_vector<double, 3> base_location = zero_vector<double>(3);
            base_location[0] = double(idx)*10.0;

            c_vector<double, 3> tip_location = zero_vector<double>(3);
            tip_location[0] = double(idx)*10.0;
            tip_location[1] = 10.0;

            p_network->FormSprout(base_location, tip_location);
        }

        // make sure vessels are correctly divided
        std::vector<boost::shared_ptr<Vessel<3> > > vessels = p_network->GetVessels();
        TS_ASSERT_EQUALS(vessels.size(), 3u);
        for(unsigned idx=0; idx<vessels.size(); idx++)
        {
            TS_ASSERT_DELTA(vessels[idx]->GetLength()/vessels[idx]->GetStartNode()->GetReferenceLengthScale(), 10.0, 1.e-6);
        }
        OutputFileHandler output_file_handler("TestVesselNetwork",false);
        p_network->Write(output_file_handler.GetOutputDirectoryFullPath() + "/multisprout.vtp");
   }
};

#endif /*TESTVESSELNETWORK_HPP_*/
