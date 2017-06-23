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



#ifndef TIPATTRACTIONLATTICEBASEDMIGRATIONRULE_HPP_
#define TIPATTRACTIONLATTICEBASEDMIGRATIONRULE_HPP_

#include <vector>
#include <string>
#include "VesselNode.hpp"
#include "SmartPointers.hpp"
#include "LatticeBasedMigrationRule.hpp"
#include "RegularGrid.hpp"
#include "UnitCollection.hpp"

/**
 * A concrete migration rule for lattice based simulations with proximity based
 * mechanical attraction between tips.
 */
template<unsigned DIM>
class TipAttractionLatticeBasedMigrationRule : public LatticeBasedMigrationRule<DIM>
{

protected:

    /**
     * Cell motility for random walks
     */
    units::quantity<unit::diffusivity> mCellMotility;

    /**
     * Cell chemotactic sensitivity
     */
    units::quantity<unit::diffusivity_per_concentration> mCellChemotacticParameter;

    /**
     * Vegf field sampled on the vessel lattice
     */
    std::vector<units::quantity<unit::concentration> > mVegfField;

    /**
     * The tip attraction radius
     */
    units::quantity<unit::length> mTipAttractionRadius;

    /**
     * The tip attraction angle
     */
    double mTipAttractionAngle;

    /**
     * The tip sprout attraction strength
     */
    units::quantity<unit::velocity> mTipSproutAttractionStrength;

    /**
     * The tip tip attraction strength
     */
    units::quantity<unit::velocity> mTipTipAttractionStrength;

    /**
     * Turn tip attraction on or off
     */
    bool mUseTipAttraction;

public:

    /**
     * Constructor.
     */
    TipAttractionLatticeBasedMigrationRule();

    /**
     * Destructor.
     */
    virtual ~TipAttractionLatticeBasedMigrationRule();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new instance of the class
     */
    static std::shared_ptr<TipAttractionLatticeBasedMigrationRule<DIM> > Create();

    /**
     * Calculate the grid index that each migrating node will move into. Set to -1 if the
     * node does not move.
     * @param rNodes nodes to calculate indices
     * @return a vector of grid indices to move nodes into
     */
    std::vector<int> GetIndices(const std::vector<std::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Set the cell chemotactic parameter
     * @param cellChemotacticParameter the cell chemotactic parameter
     */
    void SetCellChemotacticParameter(units::quantity<unit::diffusivity_per_concentration> cellChemotacticParameter);

    /**
     * Set the cell motility parameter
     * @param cellMotility the cell motility parameter
     */
    void SetCellMotilityParameter(units::quantity<unit::diffusivity> cellMotility);

    /**
     * Set whether to use tip attraction
     * @param useTipAttraction whether to use tip attraction
     */
    void SetUseTipAttraction(bool useTipAttraction);

private:

    /**
     * Get the probabilities for movement into each lattice point in the node's neighbourhood. This
     * can be over-written for custom movement rules.
     * @param pNode the sprouting node
     * @param neighbourIndices the grid indices of the neighbour nodes
     * @param gridIndex the grid index
     * @return a vector of movement probabilities corresponding to each neighbour index
     */
    std::vector<double> GetNeighbourMovementProbabilities(std::shared_ptr<VesselNode<DIM> > pNode,
                                                           std::vector<unsigned> neighbourIndices, unsigned gridIndex);
};

#endif /* TipAttractionLatticeBasedMigrationRule_HPP_ */
