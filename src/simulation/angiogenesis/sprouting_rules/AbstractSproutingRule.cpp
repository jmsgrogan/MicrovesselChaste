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

#include "AbstractSproutingRule.hpp"
#include "RandomNumberGenerator.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"

template<unsigned DIM>
AbstractSproutingRule<DIM>::AbstractSproutingRule()
    :mpSolver(),
     mSproutingProbabilityPerCell(0.00025 *unit::per_minute),
     mpVesselNetwork(),
     mUseLateralInhibition(false),
     mUseVesselEndCutoff(false),
     mOnlySproutIfPerfused(false)
{

}

template<unsigned DIM>
AbstractSproutingRule<DIM>::~AbstractSproutingRule()
{

}

template<unsigned DIM>
QRate AbstractSproutingRule<DIM>::GetSproutingProbability()
{
    return mSproutingProbabilityPerCell;
}

template<unsigned DIM>
void AbstractSproutingRule<DIM>::SetDiscreteContinuumSolver(std::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pSolver)
{
    mpSolver = pSolver;
}

template<unsigned DIM>
void AbstractSproutingRule<DIM>::SetOnlySproutIfPerfused(bool onlySproutIfPerfused)
{
    mOnlySproutIfPerfused = onlySproutIfPerfused;
}

template<unsigned DIM>
void AbstractSproutingRule<DIM>::SetSproutingProbability(QRate probability)
{
    mSproutingProbabilityPerCell = probability;
}

template<unsigned DIM>
void AbstractSproutingRule<DIM>::SetUseVesselEndCutoff(bool cutoff)
{
    mUseVesselEndCutoff = cutoff;
}

template<unsigned DIM>
void AbstractSproutingRule<DIM>::SetUseLateralInhibition(bool useInhibition)
{
    mUseLateralInhibition = useInhibition;
}

template<unsigned DIM>
void AbstractSproutingRule<DIM>::SetVesselNetwork(VesselNetworkPtr<DIM> pVesselNetwork)
{
    mpVesselNetwork = pVesselNetwork;
}

template<unsigned DIM>
void AbstractSproutingRule<DIM>::SetGridCalculator(std::shared_ptr<GridCalculator<DIM> > pGrid)
{
    // Don't do anything here, this is actually set in some child classes.
}

// Explicit instantiation
template class AbstractSproutingRule<2>;
template class AbstractSproutingRule<3>;
