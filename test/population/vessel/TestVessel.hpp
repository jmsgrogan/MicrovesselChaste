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

#ifndef TESTVESSEL_HPP_
#define TESTVESSEL_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "ChastePoint.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestVessel : public CxxTest::TestSuite
{
public:

    void TestConstructor() throw (Exception)
    {
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes;
        for (unsigned idx = 0; idx < 6; idx++)
        {
            nodes.push_back(VesselNode<2>::Create(double(idx), double(idx) + 1.0));
        }

        // Make some segments
        std::vector<boost::shared_ptr<VesselSegment<2> > > segments;
        for (unsigned idx = 0; idx < 3; idx++)
        {
            segments.push_back(VesselSegment<2>::Create(nodes[idx], nodes[idx+1]));
        }
        segments.push_back(VesselSegment<2>::Create(nodes[4], nodes[5]));

        // Make some vessels
        boost::shared_ptr<Vessel<2> > pVessel1 = Vessel<2>::Create(segments[0]);

        std::vector<boost::shared_ptr<VesselSegment<2> > > good_segments;
        good_segments.push_back(segments[1]);
        good_segments.push_back(segments[2]);

        boost::shared_ptr<Vessel<2> > pVessel2 = Vessel<2>::Create(good_segments);

        std::vector<boost::shared_ptr<VesselSegment<2> > > bad_segments = good_segments;
        bad_segments.push_back(segments[3]);
        TS_ASSERT_THROWS_THIS(Vessel<2>::Create(bad_segments), "Input vessel segments are not attached in the correct order.");
        boost::shared_ptr<Vessel<2> > pVessel3 = Vessel<2>::Create(nodes);

        // Check that locations are correct
        TS_ASSERT(pVessel1->GetStartNode()->IsCoincident(nodes[0]->rGetLocation()));
        TS_ASSERT(pVessel2->GetEndNode()->IsCoincident(nodes[3]->rGetLocation()));

        // Check that segments are correctly returned
        TS_ASSERT_EQUALS(pVessel2->GetNumberOfSegments(), 2u);
        TS_ASSERT_EQUALS(pVessel3->GetNumberOfSegments(), 5u);
        TS_ASSERT_EQUALS(pVessel2->GetSegments().size(), 2u);
        TS_ASSERT(pVessel2->GetSegments()[0]->GetNode(0)->IsCoincident(nodes[1]->rGetLocation()));

        // Test simple Getters and Setters
        pVessel1->SetId(5u);
        TS_ASSERT_EQUALS(pVessel1->GetId(), 5u);

        pVessel1->SetRadius(5.0 * 1.e-6 * unit::metres);
        pVessel1->GetFlowProperties()->SetHaematocrit(10.0);
        pVessel1->GetFlowProperties()->SetFlowRate(15.0 * unit::metre_cubed_per_second);
        TS_ASSERT_DELTA(pVessel1->GetRadius()/ (1.e-6 * unit::metres), 5.0, 1.e-6);
        TS_ASSERT_DELTA(pVessel1->GetFlowProperties()->GetHaematocrit(), 10.0, 1.e-6);
        TS_ASSERT_DELTA(pVessel1->GetFlowProperties()->GetFlowRate()/unit::metre_cubed_per_second, 15.0, 1.e-6);
    }

    void TestAddingAndRemovingSegments() throw (Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<2> > > nodes;
        for (unsigned idx = 0; idx < 6; idx++)
        {
            nodes.push_back(VesselNode<2>::Create(double(idx), double(idx) + 1.0));
        }

        // Make some segments
        std::vector<boost::shared_ptr<VesselSegment<2> > > segments;
        for (unsigned idx = 0; idx < 3; idx++)
        {
            segments.push_back(VesselSegment<2>::Create(nodes[idx], nodes[idx+1]));
        }
        segments.push_back(VesselSegment<2>::Create(nodes[4], nodes[5]));

        // Make a vessel
        boost::shared_ptr<Vessel<2> > pVessel1 = Vessel<2>::Create(segments[1]);

        // Try adding a segment to the start and end
        pVessel1->AddSegment(segments[0]);
        pVessel1->AddSegment(segments[2]);
        TS_ASSERT_EQUALS(pVessel1->GetNumberOfSegments(), 3u);

        // Try adding a disconnected segment
        TS_ASSERT_THROWS_THIS(pVessel1->AddSegment(segments[3]),
                              "Input vessel segment does not coincide with any end of the multi-segment vessel.");

        // Remove the segments from the ends
        pVessel1->RemoveSegments(SegmentLocation::Start);
        pVessel1->RemoveSegments(SegmentLocation::End);
        TS_ASSERT_EQUALS(pVessel1->GetNumberOfSegments(), 1u);

        // Vector version of adding segments
        std::vector<boost::shared_ptr<VesselSegment<2> > > good_segments;
        good_segments.push_back(segments[1]);
        good_segments.push_back(segments[2]);

        std::vector<boost::shared_ptr<VesselSegment<2> > > bad_segments = good_segments;
        bad_segments.push_back(segments[3]);
        boost::shared_ptr<Vessel<2> > pVessel2 = Vessel<2>::Create(segments[0]);
        pVessel2->AddSegments(good_segments);
        TS_ASSERT_EQUALS(pVessel2->GetNumberOfSegments(), 3u);

        boost::shared_ptr<Vessel<2> > pVessel3 = Vessel<2>::Create(segments[0]);
        TS_ASSERT_THROWS_THIS(pVessel3->AddSegments(bad_segments),
                              "Input vessel segments are not attached in the correct order.");
    }

    void TestRemoveMethod() throw (Exception)
    {
        // Make a segment
        boost::shared_ptr<VesselNode<3> > p_node1 = VesselNode<3>::Create(0.0);
        boost::shared_ptr<VesselNode<3> > p_node2 = VesselNode<3>::Create(1.0);
        boost::shared_ptr<VesselSegment<3> > pSegment1 = VesselSegment<3>::Create(p_node1, p_node2);

        // Make a vessel
        boost::shared_ptr<Vessel<3> > pVessel1 = Vessel<3>::Create(pSegment1);

        // Delete the vessel
        pVessel1->Remove();
        TS_ASSERT_EQUALS(p_node1->GetNumberOfSegments(), 0u);
        TS_ASSERT_EQUALS(p_node2->GetNumberOfSegments(), 0u);
    }
};

#endif /*TESTVessel_HPP_*/
