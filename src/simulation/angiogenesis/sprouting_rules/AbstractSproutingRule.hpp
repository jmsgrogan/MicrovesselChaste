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



#ifndef ABSTRACTSPROUTINGRULE_HPP_
#define ABSTRACTSPROUTINGRULE_HPP_

#include <vector>
#include <string>
#include "VesselNetwork.hpp"
#include "VesselNode.hpp"
#include "SmartPointers.hpp"
#include "GridCalculator.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "UnitCollection.hpp"

/**
 * Abstract class for implementing sprouting rules in angiogenesis solver.
 * Child classes implement GetSprouts() which returns a vector of potential sprouting nodes
 */
template<unsigned DIM>
class AbstractSproutingRule
{

protected:

    /**
     * A DiscreteContinuum solver containing a solution field of interest
     */
    boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > mpSolver;

    /**
     * The probability that a sprout will form per unit time
     */
    units::quantity<unit::rate> mSproutingProbability;

    /**
     * Vessel network, useful if sprouting depends on neighbouring nodes
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

    /**
     * How far from vessel ends can sprouts form
     */
    units::quantity<unit::length> mVesselEndCutoff;

public:

    /**
     * Constructor.
     */
    AbstractSproutingRule();

    /**
     * Destructor.
     */
    virtual ~AbstractSproutingRule();

    /**
     * Set the DiscreteContinuum solver containing the VEGF field
     * @param pSolver the DiscreteContinuum solver containing the VEGF field
     */
    void SetDiscreteContinuumSolver(boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pSolver);

    /**
     * Set the vessel network
     * @param pVesselNetwork pointer to a new method for the class
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork);

    /**
     * Set the sprouting probability
     * @param probability probability of sprouting per unit time
     */
    void SetSproutingProbability(units::quantity<unit::rate> probability);

    /**
     * Set the minimum distance to a vessel end at which sprouting can occur
     * @param cutoff the vessel end cutoff
     */
    void SetVesselEndCutoff(units::quantity<unit::length> cutoff);

    /**
     * Return the nodes which may form sprouts
     * @param rNodes nodes to check for sprouting
     * @return a vector of nodes which may sprout
     */
    virtual std::vector<boost::shared_ptr<VesselNode<DIM> > > GetSprouts(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Set a grid for the vessel network, implemented in some, but not all, child classes.
     * @param pGrid the grid for the vessel network
     */
    virtual void SetGridCalculator(boost::shared_ptr<GridCalculator<DIM> > pGrid);
};

#endif /* ABSTRACTSPROUTINGRULE_HPP_ */
