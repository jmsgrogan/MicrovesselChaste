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

#ifndef TESTVESSELNETWORKCALCULATORS_HPP
#define TESTVESSELNETWORKCALCULATORS_HPP

#include <cxxtest/TestSuite.h>
#include <SmartPointers.hpp>
#include "ShrinkingStimulusCalculator.hpp"
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVesselNetworkCalculators : public CxxTest::TestSuite
{

public:

    void TestShrinkingStimulusCalculator()
    {
        std::vector<std::shared_ptr<VesselNode<3> > > nodes;
        nodes.push_back(VesselNode<3>::Create(0));
        nodes.push_back(VesselNode<3>::Create(100));
        std::shared_ptr<Vessel<3> > p_vessel(Vessel<3>::Create(VesselSegment<3>::Create(nodes[0], nodes[1])));
        std::shared_ptr<VesselNetwork<3> > p_vascular_network = VesselNetwork<3>::Create();
        p_vascular_network->AddVessel(p_vessel);

        ShrinkingStimulusCalculator<3> calculator = ShrinkingStimulusCalculator<3>();
        calculator.SetVesselNetwork(p_vascular_network);
        calculator.Calculate();
    }
};

#endif // TESTVESSELNETWORKCALCULATORS_HPP
