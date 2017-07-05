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



#include "RandomNumberGenerator.hpp"
#include "VesselSegment.hpp"
#include "Vessel.hpp"
#include "LatticeBasedSproutingRule.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
LatticeBasedSproutingRule<DIM>::LatticeBasedSproutingRule()
    : AbstractSproutingRule<DIM>(),
      mpGridCalculator(),
      mTipExclusionRadius(0.0 * unit::metres)
{

}

template <unsigned DIM>
std::shared_ptr<LatticeBasedSproutingRule<DIM> > LatticeBasedSproutingRule<DIM>::Create()
{
    return std::make_shared<LatticeBasedSproutingRule<DIM> >();

}

template<unsigned DIM>
LatticeBasedSproutingRule<DIM>::~LatticeBasedSproutingRule()
{

}

template<unsigned DIM>
void LatticeBasedSproutingRule<DIM>::SetGridCalculator(std::shared_ptr<GridCalculator<DIM> > pGrid)
{
    mpGridCalculator = pGrid;
}

template<unsigned DIM>
void LatticeBasedSproutingRule<DIM>::SetTipExclusionRadius(QLength tipExclusionRadius)
{
    mTipExclusionRadius = tipExclusionRadius;
}

template<unsigned DIM>
std::vector<std::shared_ptr<VesselNode<DIM> > > LatticeBasedSproutingRule<DIM>::GetSprouts(const std::vector<std::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    if(!this->mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required for this type of sprouting rule.");
    }

    // Set up the output sprouts vector
    std::vector<std::shared_ptr<VesselNode<DIM> > > sprouts;

    // Loop over all nodes and randomly select sprouts
    for(unsigned idx = 0; idx < rNodes.size(); idx++)
    {
        if(rNodes[idx]->GetNumberOfSegments() != 2)
        {
            continue;
        }

        // Check we are not too close to the end of the vessel
        if(this->mVesselEndCutoff > 0.0 * unit::metres)
        {
            if(rNodes[idx]->GetSegment(0)->GetVessel()->GetClosestEndNodeDistance(rNodes[idx]->rGetLocation())< this->mVesselEndCutoff)
            {
                continue;
            }
            if(rNodes[idx]->GetSegment(1)->GetVessel()->GetClosestEndNodeDistance(rNodes[idx]->rGetLocation())< this->mVesselEndCutoff)
            {
                continue;
            }
        }

        // Check we are not too close to an existing candidate
        if(mTipExclusionRadius>0.0 * unit::metres)
        {
            bool too_close = false;
            for(unsigned jdx=0; jdx<sprouts.size(); jdx++)
            {
                if(rNodes[idx]->GetDistance(sprouts[jdx]->rGetLocation()) < mTipExclusionRadius)
                {
                    too_close = true;
                }
            }
            if(too_close)
            {
                continue;
            }
        }

        double prob_tip_selection = this->mSproutingProbability*SimulationTime::Instance()->GetTimeStep()*BaseUnits::Instance()->GetReferenceTimeScale();
        if (RandomNumberGenerator::Instance()->ranf() < prob_tip_selection)
        {
            sprouts.push_back(rNodes[idx]);
        }
    }
    return sprouts;
}

// Explicit instantiation
template class LatticeBasedSproutingRule<2> ;
template class LatticeBasedSproutingRule<3> ;
