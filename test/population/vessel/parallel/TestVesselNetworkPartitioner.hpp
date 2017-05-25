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

#ifndef TESTVESSELNETWORKPARTITIONER_HPP_
#define TESTVESSELNETWORKPARTITIONER_HPP_

#include <cxxtest/TestSuite.h>
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "VesselNetworkGenerator.hpp"
#include "VesselNetworkPartitioner.hpp"

#include "PetscAndVtkSetupAndFinalize.hpp"

class TestVesselNetworkPartitioner : public CxxTest::TestSuite
{
public:

    void TestConstructor() throw (Exception)
    {
        boost::shared_ptr<VesselNode<2> > pnode_1 = VesselNode<2>::Create();
        boost::shared_ptr<VesselNode<2> > pnode_2 = VesselNode<2>::Create();
        boost::shared_ptr<VesselNode<2> > pnode_3 = VesselNode<2>::Create();

        // Make some vessels
        boost::shared_ptr<Vessel<2> > pVessel1 = Vessel<2>::Create(pnode_1, pnode_2);
        boost::shared_ptr<Vessel<2> > pVessel2 = Vessel<2>::Create(pnode_2, pnode_3);

        boost::shared_ptr<VesselNetwork<2> > p_network = VesselNetwork<2>::Create();

        VesselNetworkPartitioner<2> partitioner;
        partitioner.SetVesselNetwork(p_network);
        partitioner.Update();
    }
};

#endif /*TESTVESSELNETWORKPARTITIONER_HPP_*/
