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

#ifndef TESTVESSELNODE_HPP_
#define TESTVESSELNODE_HPP_

#include "CheckpointArchiveTypes.hpp"
#include "ArchiveLocationInfo.hpp"
#include <boost/serialization/shared_ptr.hpp>
#include "Exception.hpp"
#include "AbstractCellBasedTestSuite.hpp"
#include "ChastePoint.hpp"
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "VesselSegment.hpp"
#include "VesselNode.hpp"
#include "NodeFlowProperties.hpp"
#include "UnitCollection.hpp"
#include "OutputFileHandler.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestVesselNode : public AbstractCellBasedTestSuite
{

public:

    void TestConstructorAndLocationMethods() throw (Exception)
    {
        // Set up some points and locations
        ChastePoint<3> point1(1.0, 2.0, 3.0);
        ChastePoint<2> point2(5.0, 6.0);

        c_vector<double, 3> location1;
        location1[0] = 1.0;
        location1[1] = 2.0;
        location1[2] = 0.0;

        // Pointer Factory Constructors
        auto p_node_1 = VesselNode<3>::Create(2_um, 3_um, 4_um);
        auto p_node_3 = VesselNode<3>::Create(Vertex<3>(location1, 1_um));

        // Test the location methods
        TS_ASSERT_DELTA(p_node_1->rGetLocation().Convert(1_um)[0], 2.0, 1.e-6);
        TS_ASSERT_DELTA(p_node_1->rGetLocation().Convert(1_um)[1], 3.0, 1.e-6);
        TS_ASSERT_DELTA(p_node_1->rGetLocation().Convert(1_um)[2], 4.0, 1.e-6);
    }

    void TestSimpleGetAndSetMethods() throw (Exception)
    {
        // Make a node
        std::shared_ptr<VesselNode<3> > p_node = VesselNode<3>::Create(0_m);

        // Test simple Getters and Setters
        p_node->SetId(5u);
        p_node->GetFlowProperties()->SetPressure(5.0 * unit::pascals);
        p_node->SetRadius(10.e-6 * unit::metres);
        p_node->GetFlowProperties()->SetIsInputNode(true);
        p_node->GetFlowProperties()->SetIsOutputNode(true);

        TS_ASSERT_EQUALS(p_node->GetId(), 5u);
        TS_ASSERT_DELTA(p_node->GetFlowProperties()->GetPressure() / unit::pascals, 5.0 , 1.e-6);
        TS_ASSERT_DELTA(p_node->GetRadius()/(1_um), 10.0, 1.e-6);
        TS_ASSERT(p_node->GetFlowProperties()->IsInputNode());
        TS_ASSERT(p_node->GetFlowProperties()->IsOutputNode());

        // Test setting node flow properties
        NodeFlowProperties<3> node_flow_properties;
        node_flow_properties.SetPressure(12.0 * unit::pascals);
        p_node->SetFlowProperties(node_flow_properties);
    }

    void TestDistanceAndConincidentMethods() throw (Exception)
    {
        // Set up some points nodes
        auto p_node_1 = VesselNode<3>::Create(1.0_um, 2.0_um, 3.0_um);
        auto p_node_2 = VesselNode<3>::Create(1.0_um, 2.0_um, 3.0_um);
        auto p_node_3 = VesselNode<3>::Create(4.0_um, 5.0_um, 6.0_um);

        c_vector<double, 3> location1;
        location1[0] = 1.0;
        location1[1] = 2.0;
        location1[2] = 3.0;

        // Coincident methods
        TS_ASSERT(p_node_1->IsCoincident(p_node_2->rGetLocation()));
        TS_ASSERT(p_node_1->IsCoincident(Vertex<3>(location1, 1_um)));

        // Distance methods
        TS_ASSERT_DELTA(p_node_1->GetDistance(p_node_3->rGetLocation())/1_um, std::sqrt(27.0), 1.e-6);
    }

    void TestAddingAndRemovingVesselSegments() throw (Exception)
    {
        // Make some nodes
        auto p_node_1 = VesselNode<2>::Create(0_um);
        auto p_node_2 = VesselNode<2>::Create(1_um);
        auto p_node_3 = VesselNode<2>::Create(2_um);

        // Make some vessel segments
        auto p_segment1 = VesselSegment<2>::Create(p_node_1, p_node_2);
        auto p_segment2 = VesselSegment<2>::Create(p_node_2, p_node_3);

        // Check that the vessel segments have been suitably added to the nodes.
        TS_ASSERT(p_node_1->IsAttachedTo(p_segment1));
        TS_ASSERT(!p_node_3->IsAttachedTo(p_segment1));
        TS_ASSERT_EQUALS(p_node_1->GetNumberOfSegments(), 1u);
        TS_ASSERT_EQUALS(p_node_2->GetNumberOfSegments(), 2u);

        // Check that the segments are correctly retrieved from the node.
        TS_ASSERT(p_node_2->IsCoincident(p_node_2->GetSegment(0)->GetNode(1)->rGetLocation()));
        TS_ASSERT(p_node_2->IsCoincident(p_node_2->GetSegments()[0]->GetNode(1)->rGetLocation()));
        TS_ASSERT_THROWS_THIS(p_node_2->GetSegment(3), "Requested segment index out of range");

        // Check that the vessel segment connectivity is updated when a node is replaced.
        p_segment2->ReplaceNode(1, p_node_1);
        TS_ASSERT_EQUALS(p_node_1->GetNumberOfSegments(), 2u);
        TS_ASSERT_EQUALS(p_node_3->GetNumberOfSegments(), 0u);

        // Check that a node can't be replaced with one that's already there
        TS_ASSERT_THROWS_THIS(p_segment2->ReplaceNode(0, p_node_1), "This segment is already attached to this node.");
    }

    void TestArchiving() throw (Exception)
    {
        // Test Archiving
        OutputFileHandler handler("archive", false);
        ArchiveLocationInfo::SetArchiveDirectory(handler.FindFile(""));
        std::string archive_filename = ArchiveLocationInfo::GetProcessUniqueFilePath("VesselNode.arch");

        // Save archive
        {
            std::shared_ptr<VesselNode<3> > p_node = std::shared_ptr<VesselNode<3> >(new VesselNode<3>(1_m, 2.0_m, 3.0_m));
            auto p_cast_node = std::static_pointer_cast<AbstractVesselNetworkComponent<3> >(p_node);
            TS_ASSERT_DELTA(p_node->rGetLocation().Convert(1.0*unit::metres)[0], 1.0, 1.e-6);
            TS_ASSERT_DELTA(p_node->rGetLocation().Convert(1.0*unit::metres)[1], 2.0, 1.e-6);
            TS_ASSERT_DELTA(p_node->rGetLocation().Convert(1.0*unit::metres)[2], 3.0, 1.e-6);

            std::ofstream ofs(archive_filename.c_str());
            boost::archive::text_oarchive output_arch(ofs);
            output_arch << p_cast_node;
        }

        // Load archive
        {
            std::shared_ptr<AbstractVesselNetworkComponent<3> > p_node_from_archive;

            // Read from this input file
            std::ifstream ifs(archive_filename.c_str(), std::ios::binary);
            boost::archive::text_iarchive input_arch(ifs);

            // restore from the archive
            input_arch >> p_node_from_archive;
            auto p_cast_node = std::static_pointer_cast<VesselNode<3> >(p_node_from_archive);
            TS_ASSERT_DELTA(p_cast_node->rGetLocation().Convert(1.0*unit::metres)[0], 1.0, 1.e-6);
            TS_ASSERT_DELTA(p_cast_node->rGetLocation().Convert(1.0*unit::metres)[1], 2.0, 1.e-6);
            TS_ASSERT_DELTA(p_cast_node->rGetLocation().Convert(1.0*unit::metres)[2], 3.0, 1.e-6);
        }
    }
};

#endif /*TESTVESSELNODE_HPP_*/
