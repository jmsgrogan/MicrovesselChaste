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

#ifndef TESTVESSELNETWORKGEOMETRYCALCULATOR_HPP_
#define TESTVESSELNETWORKGEOMETRYCALCULATOR_HPP_

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
#include "VesselNetworkGeometryCalculator.hpp"

#include "PetscSetupAndFinalize.hpp"

class TestVesselNetworkGeometryCalculator : public CxxTest::TestSuite
{
public:

    void TestSimpleNetwork() throw(Exception)
    {
        // Make some nodes
        std::vector<boost::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(1.0, 2.0, 6.0));
        nodes.push_back(VesselNode<3>::Create(3.0, 4.0, 7.0));
        nodes.push_back(VesselNode<3>::Create(3.0, 4.0, 7.0));
        nodes.push_back(VesselNode<3>::Create(3.0, 4.0, 8.0));
        nodes.push_back(VesselNode<3>::Create(3.0, 4.0, 9.0));

        // Make some vessels
        boost::shared_ptr<Vessel<3> > pVessel1(Vessel<3>::Create(nodes[0], nodes[1]));
        boost::shared_ptr<Vessel<3> > pVessel2(Vessel<3>::Create(nodes[2], nodes[3]));
        boost::shared_ptr<Vessel<3> > pVessel3(Vessel<3>::Create(nodes[3], nodes[4]));

        std::vector<boost::shared_ptr<Vessel<3> > > vessels;
        vessels.push_back(pVessel2);
        vessels.push_back(pVessel3);

        // Make a network
        boost::shared_ptr<VesselNetwork<3> > p_network = VesselNetwork<3>::Create();
        p_network->AddVessel(pVessel1);
        p_network->AddVessels(vessels);

        boost::shared_ptr<VesselNetworkGeometryCalculator<3> > p_calculator = VesselNetworkGeometryCalculator<3>::Create();
        p_calculator->SetVesselNetwork(p_network);

        TS_ASSERT_DELTA(p_calculator->GetAverageInterSegmentDistance().value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(p_calculator->GetAverageVesselLength().value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(p_calculator->GetTotalLength().value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(p_calculator->GetTotalSurfaceArea().value(), 0.0, 1.e-6);
        TS_ASSERT_DELTA(p_calculator->GetTotalVolume().value(), 0.0, 1.e-6);
    }
};

#endif /*TESTVESSELNETWORKGEOMETRYCALCULATOR_HPP_*/
