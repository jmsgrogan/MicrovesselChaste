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

#ifndef VESSELNETWORKPARTITIONER_HPP_
#define VESSELNETWORKPARTITIONER_HPP_

#include <vector>
#include <string>
#include <map>
#include "UblasVectorInclude.hpp"
#include "UnitCollection.hpp"
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"

/**
 * Partition a vessel network on master over the available processors.
 */
template<unsigned DIM>
class VesselNetworkPartitioner
{

    /**
     * The network to be partitioned
     */
    std::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * Partition into approx equal divisions along the specified axis
     */
    bool mUseSimpleGeometricPartition;

    /**
     * Axis for partitioning
     * 0 - x
     * 1 - y
     * 2 - z
     */
    unsigned mParitionAxis;

public:

    /**
     * Constructor.
     */
    VesselNetworkPartitioner();

    /**
     * Destructor
     */
     ~VesselNetworkPartitioner();

     /**
      * Use simple geometric partitioning (default)
      * @param useSimple whether to use simple geometric partitioning
      */
    void SetUseSimpleGeometricPartitioning(bool useSimple);

    /**
     * Set the partition axis, 0 - x, 1 - y, 2 - z
     * @param partitionAxis the partition axis, 0 - x, 1 - y, 2 - z
     */
    void SetPartitionAxis(unsigned partitionAxis);

    /**
     * Set the vessel network
     *
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Do the partitioning
     */
    void Update();

};

#endif /* VESSELNETWORKPARTITIONER_HPP_ */
