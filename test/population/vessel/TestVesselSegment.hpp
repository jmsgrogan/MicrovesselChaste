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

#ifndef TESTVESSELSEGMENT_HPP_
#define TESTVESSELSEGMENT_HPP_

#include <cxxtest/TestSuite.h>
#include <math.h>
#include "VesselNode.hpp"
#include "SmartPointers.hpp"
#include "ChastePoint.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "UnitCollection.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestVesselSegment : public CxxTest::TestSuite
{
public:

    void TestConstructor() throw (Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes;
        nodes.push_back(VesselNode<2>::Create(1.0, 2.0));
        nodes.push_back(VesselNode<2>::Create(3.0, 4.0));

        // Check for an exception if the segment is defined with the same nodes
        TS_ASSERT_THROWS_THIS(VesselSegment<2>::Create(nodes[0], nodes[0]),
                              "Attempted to assign the same node to both ends of a vessel segment.");

        // Make a segment
        boost::shared_ptr<VesselSegment<2> > p_segment = VesselSegment<2>::Create(nodes[0], nodes[1]);

        // Check the locations
        TS_ASSERT(p_segment->GetNodes().first->IsCoincident(nodes[0]->rGetLocation()));
        TS_ASSERT(p_segment->GetNode(0)->IsCoincident(nodes[0]->rGetLocation()));
        TS_ASSERT(p_segment->GetNodes().second->IsCoincident(nodes[1]->rGetLocation()));
        TS_ASSERT(p_segment->GetNode(1)->IsCoincident(nodes[1]->rGetLocation()));
        TS_ASSERT_THROWS_THIS(p_segment->GetNode(2),
                              "A node index other than 0 or 1 has been requested for a Vessel Segment.");

        // Test replacing a node
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes2;
        nodes2.push_back(VesselNode<2>::Create(6.0, 7.0));
        nodes2.push_back(VesselNode<2>::Create(8.0, 9.0));

        p_segment->ReplaceNode(0, nodes2[0]);
        p_segment->ReplaceNode(1, nodes2[1]);
        TS_ASSERT(p_segment->GetNode(0)->IsCoincident(nodes2[0]->rGetLocation()));
        TS_ASSERT(p_segment->GetNode(1)->IsCoincident(nodes2[1]->rGetLocation()));
        TS_ASSERT_THROWS_THIS(p_segment->ReplaceNode(2, nodes2[0]),
                              "A node index other than 0 or 1 has been requested for a Vessel Segment.");
    }

    void TestSimpleGetAndSetMethods() throw (Exception)
    {
        boost::shared_ptr<VesselSegment<3> > pSegment = VesselSegment<3>::Create(VesselNode<3>::Create(),
                                                                                      VesselNode<3>::Create(1.0));

        // Test simple Getters and Setters
        pSegment->SetId(5u);
        TS_ASSERT_EQUALS(pSegment->GetId(), 5u);

        pSegment->SetRadius(5.0e-6 * unit::metres);
        TS_ASSERT_DELTA(pSegment->GetRadius()/(1.e-6*unit::metres), 5.0, 1.e-6);

        pSegment->GetFlowProperties()->SetHaematocrit(10.0);
        pSegment->GetFlowProperties()->SetFlowRate(15.0 * unit::metre_cubed_per_second);
        TS_ASSERT_DELTA(pSegment->GetFlowProperties()->GetHaematocrit(), 10.0, 1.e-6);
        TS_ASSERT_DELTA(pSegment->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, 15.0, 1.e-6);
    }

    void TestGeometricFeatures() throw (Exception)
    {
        //Check the returned length
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes;
        nodes.push_back(VesselNode<2>::Create(6.0, 7.0));
        nodes.push_back(VesselNode<2>::Create(8.0, 9.0));

        std::vector<boost::shared_ptr<VesselNode<3> > > nodes_3d;
        nodes_3d.push_back(VesselNode<3>::Create(3.0, 4.0, 5.0));
        nodes_3d.push_back(VesselNode<3>::Create(6.0, 7.0, 8.0));

        boost::shared_ptr<VesselSegment<2> > p_segment1 = VesselSegment<2>::Create(nodes[0], nodes[1]);
        boost::shared_ptr<VesselSegment<3> > p_segment2 = VesselSegment<3>::Create(nodes_3d[0], nodes_3d[1]);

        TS_ASSERT_DELTA(p_segment1->GetLength()/p_segment1->GetNode(0)->GetReferenceLengthScale(), std::sqrt(8.0), 1.e-6);
        TS_ASSERT_DELTA(p_segment2->GetLength()/p_segment2->GetNode(0)->GetReferenceLengthScale(), std::sqrt(27.0), 1.e-6);

        // Test point distance calculation
        TS_ASSERT_DELTA(p_segment1->GetDistance(nodes[0]->rGetLocation())/p_segment1->GetNode(0)->GetReferenceLengthScale(), 0, 1.e-6);

        // Test Unit tangent and point projection
        ChastePoint<2> tangent(1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0));
        TS_ASSERT(ChastePoint<2>(p_segment1->GetUnitTangent()).IsSamePoint(tangent));
    }

    void TestAddingAndRemovingVessels() throw (Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes;
        nodes.push_back(VesselNode<2>::Create(4.0, 3.0));
        nodes.push_back(VesselNode<2>::Create(4.0, 5.0));
        nodes.push_back(VesselNode<2>::Create(5.0, 6.0));

        // Make some vessel segments1
        boost::shared_ptr<VesselSegment<2> > pSegment = VesselSegment<2>::Create(nodes[0], nodes[1]);
        boost::shared_ptr<VesselSegment<2> > pSegment2 = VesselSegment<2>::Create(nodes[1], nodes[2]);

        TS_ASSERT_THROWS_THIS(boost::shared_ptr<Vessel<2> > vessel = pSegment->GetVessel(),
                              "A vessel has been requested but this segment doesn't have one.");

        // Make a vessel and check that it has been suitably added to the segment
        boost::shared_ptr<Vessel<2> > pVessel = Vessel<2>::Create(pSegment);
        TS_ASSERT(pSegment->GetNode(0)->IsCoincident(pSegment->GetVessel()->GetSegments()[0]->GetNode(0)->rGetLocation()));

        // Add a different vessel
        boost::shared_ptr<Vessel<2> > pVessel2 = Vessel<2>::Create(pSegment);
        TS_ASSERT(pSegment->GetNode(0)->IsCoincident(pSegment->GetVessel()->GetSegments()[0]->GetNode(0)->rGetLocation()));

        // Try removing a segment from the vessel
        pVessel->AddSegment(pSegment2);
        pVessel->RemoveSegments(SegmentLocation::Start);
        TS_ASSERT_THROWS_THIS(pVessel->RemoveSegments(SegmentLocation::End), "Vessel must have at least one segment.");
        TS_ASSERT_THROWS_THIS(boost::shared_ptr<Vessel<2> > vessel = pSegment->GetVessel(),
                              "A vessel has been requested but this segment doesn't have one.");
    }

    void TestRemoveMethod() throw (Exception)
    {
        // Make a segment
        boost::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3>::Create(0.0);
        boost::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3>::Create(1.0);
        boost::shared_ptr<VesselSegment<3> > pSegment1 = VesselSegment<3>::Create(p_node1, p_node2);

        // Delete the segment
        pSegment1->Remove();
        TS_ASSERT_EQUALS(p_node1->GetNumberOfSegments(), 0u);
        TS_ASSERT_EQUALS(p_node2->GetNumberOfSegments(), 0u);
    }
};

#endif /*TESTVESSELSEGMENT_HPP_*/
