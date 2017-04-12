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
#include "Owen2011SproutingRule.hpp"
#include "Owen11Parameters.hpp"

#include "Debug.hpp"

template<unsigned DIM>
Owen2011SproutingRule<DIM>::Owen2011SproutingRule()
    : LatticeBasedSproutingRule<DIM>(),
      mHalfMaxVegf(Owen11Parameters::mpVegfConventrationAtHalfMaxProbSprouting->GetValue("Owen2011SproutingRule")),
      mVegfField()
{
    // Owen11 equation is dimensionally inconsistent as it includes an extra vessel surface area term. In order to
    // have a similar magnitude multiply this value by a typical vessel surface area = 2*pi*R*L
    // = 2*pi*15*40
    this->mSproutingProbability = Owen11Parameters::mpMaximumSproutingRate->GetValue("Owen2011SproutingRule")*2.0*M_PI*15.0*40.0;
    this->mTipExclusionRadius = 80.0*1.e-6*unit::metres;
    this->mVesselEndCutoff = 80.0*1.e-6* unit::metres;
}

template <unsigned DIM>
boost::shared_ptr<Owen2011SproutingRule<DIM> > Owen2011SproutingRule<DIM>::Create()
{
    MAKE_PTR(Owen2011SproutingRule<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
Owen2011SproutingRule<DIM>::~Owen2011SproutingRule()
{

}

template<unsigned DIM>
void Owen2011SproutingRule<DIM>::SetHalfMaxVegf(units::quantity<unit::concentration> halfMaxVegf)
{
    mHalfMaxVegf = halfMaxVegf;
}

template<unsigned DIM>
std::vector<boost::shared_ptr<VesselNode<DIM> > > Owen2011SproutingRule<DIM>::GetSprouts(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    MARK;
    if(!this->mpGridCalculator)
    {
        EXCEPTION("A regular grid is required for this type of sprouting rule.");
    }

    if(!this->mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required for this type of sprouting rule.");
    }
    MARK;
    // Get the VEGF field
    this->mVegfField = this->mpSolver->GetConcentrations(this->mpGridCalculator->GetGrid());
    MARK;
    // Set up the output sprouts vector
    std::vector<boost::shared_ptr<VesselNode<DIM> > > sprouts;

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
        if(this->mTipExclusionRadius>0.0 * unit::metres)
        {
            bool too_close = false;
            for(unsigned jdx=0; jdx<sprouts.size(); jdx++)
            {
                if(rNodes[idx]->GetDistance(sprouts[jdx]->rGetLocation()) < this->mTipExclusionRadius)
                {
                    too_close = true;
                }
            }
            if(too_close)
            {
                continue;
            }
        }
        MARK;
        // Get the grid index of the node
        unsigned grid_index = this->mpGridCalculator->GetGrid()->GetNearestCellIndex(rNodes[idx]->rGetLocation());
        MARK;
        units::quantity<unit::concentration> vegf_conc = this->mVegfField[grid_index];
        double vegf_fraction = vegf_conc/(vegf_conc + mHalfMaxVegf);
        double max_prob_per_time_step = this->mSproutingProbability*SimulationTime::Instance()->GetTimeStep()*BaseUnits::Instance()->GetReferenceTimeScale();
        double prob_tip_selection = max_prob_per_time_step*vegf_fraction;
        MARK;
        if (RandomNumberGenerator::Instance()->ranf() < prob_tip_selection)
        {
            sprouts.push_back(rNodes[idx]);
        }
        MARK;
    }
    return sprouts;
}

// Explicit instantiation
template class Owen2011SproutingRule<2> ;
template class Owen2011SproutingRule<3> ;
