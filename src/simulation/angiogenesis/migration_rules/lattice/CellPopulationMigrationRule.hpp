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



#ifndef CELLPOPULATIONMIGRATIONRULE_HPP_
#define CELLPOPULATIONMIGRATIONRULE_HPP_

#include <vector>
#include <string>
#include "LatticeBasedMigrationRule.hpp"
#include "AbstractCellMutationState.hpp"
#include "VesselNode.hpp"
#include "SmartPointers.hpp"
#include "AbstractCellPopulation.hpp"

/**
 * A simple random direction lattice based migration rule. Not physical, but useful for code testing.
 */
template<unsigned DIM>
class CellPopulationMigrationRule : public LatticeBasedMigrationRule<DIM>
{

protected:

    /**
     * Volume fraction of a lattice site that each cell will occupy
     */
    std::map<boost::shared_ptr<AbstractCellMutationState> , double > mVolumeFractionMap;

    /**
     * Collection of cells at each lattice point
     */
    std::vector<std::vector<CellPtr> > mPointCellMap;

public:

    /**
     * Constructor.
     */
    CellPopulationMigrationRule();

    /**
     * Destructor.
     */
    virtual ~CellPopulationMigrationRule();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new instance of the class
     */
    static std::shared_ptr<CellPopulationMigrationRule<DIM> > Create();

    /**
     * Calculate the grid index that each migrating node will move into. Set to -1 if the
     * node does not move.
     * @param rNodes nodes to calculate indices
     * @return a vector of grid indices to move nodes into
     */
    virtual std::vector<int> GetIndices(const std::vector<std::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Method to set volume fraction for particular type of cell.
     * @param mutation_state the cell muatation state
     * @param volume_fraction the occupying fraction
     */
    void SetVolumeFraction(boost::shared_ptr<AbstractCellMutationState> mutation_state, double volume_fraction);

    /**
     * Return occupying volume fraction for particular type of cell.
     * @param mutation_state the cell muatation state
     * @return the occupying fraction
     */
    double GetOccupyingVolumeFraction(boost::shared_ptr<AbstractCellMutationState> mutation_state);

protected:

    /**
     * Get the probabilities for movement into each lattice point in the node's neighbourhood. This
     * can be over-written for custom movement rules.
     * @param pNode the sprouting node
     * @param neighbourIndices the grid indices of the neighbour nodes
     * @param gridIndex the current grid index
     * @return a vector of movement probabilities corresponding to each neighbour index
     */
    virtual std::vector<double> GetNeighbourMovementProbabilities(std::shared_ptr<VesselNode<DIM> > pNode,
                                                           std::vector<unsigned> neighbourIndices, unsigned gridIndex);

};

#endif /* CELLPOPULATIONMIGRATIONRULE_HPP_ */
