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

#ifndef VESSELNETWORKGRAPHCALCULATOR_HPP_
#define VESSELNETWORKGRAPHCALCULATOR_HPP_

#include <memory>
#include <string>
#include <vector>
#include "VesselNetwork.hpp"

/**
 * Calculate graph properties of vessel networks
 */
template<unsigned DIM>
class VesselNetworkGraphCalculator
{

private:

    /**
     * Container for the VesselNetwork.
     */
    std::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

public:

    /**
     * Constructor
     */
    VesselNetworkGraphCalculator();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to the class instance
     */
    static std::shared_ptr<VesselNetworkGraphCalculator<DIM> > Create();

    /**
     * Destructor
     */
    ~VesselNetworkGraphCalculator();

    /**
     * Set the vessel network
     * @param pVesselNetwork the vessel network
     */
    void SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork);

    /**
     * Return the indices of each node attached to a node
     * @return the node-node connectivity
     */
    std::vector<std::vector<unsigned> > GetNodeNodeConnectivity();

    /**
     * Return the indices of each vessel attached to a node
     * @return the indices of each vessel attached to a node
     */
    std::vector<std::vector<unsigned> > GetNodeVesselConnectivity();

    /**
     * Return whether a node is connected to a source node.
     * @param pSourceNode the source node
     * @param pQueryNode the query node
     * @return whether a node is connected to a source node.
     */
    bool IsConnected(std::shared_ptr<VesselNode<DIM> > pSourceNode, std::shared_ptr<VesselNode<DIM> > pQueryNode);

    /**
     * Return whether a vector of nodes is connected to a vector of source nodes.
     * @param sourceNodes the source nodes
     * @param queryNodes the query nodes
     * @return whether a node is connected to a source node.
     */
    std::vector<bool> IsConnected(std::vector<std::shared_ptr<VesselNode<DIM> > > sourceNodes,
                                  std::vector<std::shared_ptr<VesselNode<DIM> > > queryNodes);

    /**
     * Outputs connectivity of vessels to file in graphviz format (.gv).
     * @param rFilename the output filename
     */
    void WriteConnectivity(const std::string& rFilename);

};

#endif /* VESSELNETWORKGRAPHCALCULATOR_HPP_ */
