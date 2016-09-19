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

#ifndef LATTICEBASEDMIGRATIONRULE_HPP_
#define LATTICEBASEDMIGRATIONRULE_HPP_

#include <vector>
#include <string>
#include "AbstractMigrationRule.hpp"
#include "VesselNode.hpp"
#include "SmartPointers.hpp"

/**
 * A simple random direction lattice based migration rule. Not physical, but useful for code testing.
 */
template<unsigned DIM>
class LatticeBasedMigrationRule : public AbstractMigrationRule<DIM>
{

protected:

    /**
     * Cell movement probability
     */
    double mMovementProbability;

public:

    /**
     * Constructor.
     */
    LatticeBasedMigrationRule();

    /**
     * Destructor.
     */
    virtual ~LatticeBasedMigrationRule();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new instance of the class
     */
    static boost::shared_ptr<LatticeBasedMigrationRule<DIM> > Create();

    /**
     * Calculate the grid index that each migrating node will move into. Set to -1 if the
     * node does not move.
     * @param rNodes nodes to calculate indices
     * @return a vector of grid indices to move nodes into
     */
    virtual std::vector<int> GetIndices(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Set the movement probability
     * @param movementProbability the movement probability
     */
    void SetMovementProbability(double movementProbability);

protected:

    /**
     * Get the probabilities for movement into each lattice point in the node's neighbourhood. This
     * can be over-written for custom movement rules.
     * @param pNode the sprouting node
     * @param neighbourIndices the grid indices of the neighbour nodes
     * @return a vector of movement probabilities corresponding to each neighbour index
     */
    virtual std::vector<double> GetNeighbourMovementProbabilities(boost::shared_ptr<VesselNode<DIM> > pNode,
                                                           std::vector<unsigned> neighbourIndices, unsigned gridIndex);

    /**
     * Get the index of the neighbour to move into.
     * This can be over-written for custom movement rules.
     * @param movementProbabilities the movement probabilities corresponding to each neighbour index
     * @param neighbourIndices the grid indices of the neighbour nodes
     * @return the neighbour index to move into
     */
    virtual int GetNeighbourMovementIndex(std::vector<double> movementProbabilities,
                                               std::vector<unsigned> neighbourIndices);
};

#endif /* LATTICEBASEDMIGRATIONRULE_HPP_ */
