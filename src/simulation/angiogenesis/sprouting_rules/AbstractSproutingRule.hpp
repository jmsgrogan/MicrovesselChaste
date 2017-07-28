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

#include <memory>
#include <vector>
#include <string>
#include "VesselNetwork.hpp"
#include "VesselNode.hpp"
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
    std::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > mpSolver;

    /**
     * The probability that ONE CELL will form a sprout per unit time
     */
    QRate mSproutingProbabilityPerCell;

    /**
     * Vessel network, useful if sprouting depends on neighbouring nodes
     */
    VesselNetworkPtr<DIM> mpVesselNetwork;

    /**
     * Whether to use lateral inhibition in the sprouting rule.
     */
    bool mUseLateralInhibition;

    /**
     * Prevents sprouting too close to branches and existing tips.
     */
    bool mUseVesselEndCutoff;

    /**
     * Only perfused vessels can sprout if true
     */
    bool mOnlySproutIfPerfused;

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
     * Return the sprouting probability per cell
     * @return the sprouting probability per cell
     */
    QRate GetSproutingProbability();

    /**
     * Return the nodes which may form sprouts
     * @param rNodes nodes to check for sprouting
     * @return a vector of nodes which may sprout
     */
    virtual std::vector<VesselNodePtr<DIM> > GetSprouts(const std::vector<VesselNodePtr<DIM> >& rNodes)=0;

    /**
     * Set the DiscreteContinuum solver containing the VEGF field
     * @param pSolver the DiscreteContinuum solver containing the VEGF field
     */
    void SetDiscreteContinuumSolver(std::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pSolver);

    /**
     * Set a grid for the vessel network, implemented in some, but not all, child classes.
     * @param pGrid the grid for the vessel network
     */
    virtual void SetGridCalculator(std::shared_ptr<GridCalculator<DIM> > pGrid);

    /**
     * Set whether only perfused vessels can sprout
     * @param onlySproutIfPerfused only perfused vessels can sprout if true
     */
    void SetOnlySproutIfPerfused(bool onlySproutIfPerfused);

    /**
     * Set the sprouting probability per cell
     * @param probability probability of sprouting per cell per unit time
     */
    void SetSproutingProbability(QRate probability);

    /**
     * Set the vessel network
     * @param pVesselNetwork pointer to a new method for the class
     */
    void SetVesselNetwork(VesselNetworkPtr<DIM> pVesselNetwork);

    /**
     * Set whether to use a vessel-end cutoff
     * @param useCutoff whether to use a vessel-end cutoff
     */
    void SetUseVesselEndCutoff(bool useCutoff);

    /**
     * Set whether to use lateral inhibition
     * @param useInhibition whether to use lateral inhibition
     */
    void SetUseLateralInhibition(bool useInhibition);

};

#endif /* ABSTRACTSPROUTINGRULE_HPP_ */
