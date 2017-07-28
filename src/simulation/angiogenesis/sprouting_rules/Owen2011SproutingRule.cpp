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

template<unsigned DIM>
Owen2011SproutingRule<DIM>::Owen2011SproutingRule()
    : LatticeBasedSproutingRule<DIM>(),
      mHalfMaxVegf(Owen11Parameters::mpVegfConventrationAtHalfMaxProbSprouting->GetValue("Owen2011SproutingRule")),
      mVegfField()
{
    this->mSproutingProbabilityPerCell = Owen11Parameters::mpMaximumSproutingRate->GetValue("Owen2011SproutingRule");
    this->mUseLateralInhibition = true;
    this->mUseVesselEndCutoff = true;
}

template <unsigned DIM>
std::shared_ptr<Owen2011SproutingRule<DIM> > Owen2011SproutingRule<DIM>::Create()
{
    return std::make_shared<Owen2011SproutingRule<DIM> >();

}

template<unsigned DIM>
Owen2011SproutingRule<DIM>::~Owen2011SproutingRule()
{

}

template<unsigned DIM>
void Owen2011SproutingRule<DIM>::SetHalfMaxVegf(QConcentration halfMaxVegf)
{
    mHalfMaxVegf = halfMaxVegf;
}

template<unsigned DIM>
std::vector<VesselNodePtr<DIM> > Owen2011SproutingRule<DIM>::GetSprouts(const std::vector<VesselNodePtr<DIM> >& rNodes)
{
    if(!this->mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required for this type of sprouting rule.");
    }

    if(!this->mpGridCalculator)
    {
        EXCEPTION("A grid calculator is required for this type of sprouting rule.");
    }

    if(!this->mpGridCalculator->HasStructuredGrid())
    {
        EXCEPTION("A regular grid is required for this type of sprouting rule.");
    }

    if(!this->mpSolver)
    {
        EXCEPTION("A concentration field is required for this sprouting rule.");
    }

    // Get the VEGF field
    this->mVegfField = this->mpSolver->GetConcentrations(this->mpGridCalculator->GetGrid());

    // Set up the output sprouts vector
    std::vector<VesselNodePtr<DIM> > sprouts;
    QLength grid_spacing = this->mpGridCalculator->GetGrid()->GetSpacing();
    QTime reference_time = BaseUnits::Instance()->GetReferenceTimeScale();

    // Loop over all nodes and randomly select sprouts
    for(auto& node:rNodes)
    {
        if(node->GetNumberOfSegments() != 2)
        {
            continue;
        }

        // Check we are not too close to the end of the vessel
        if(this->mUseVesselEndCutoff)
        {
            if(node->GetSegment(0)->GetVessel()->GetClosestEndNodeDistance(node->rGetLocation())< grid_spacing)
            {
                continue;
            }
            if(node->GetSegment(1)->GetVessel()->GetClosestEndNodeDistance(node->rGetLocation())< grid_spacing)
            {
                continue;
            }
        }

        // Check we are not too close to an existing candidate
        if(this->mUseLateralInhibition)
        {
            bool too_close = false;
            for(auto& sprout:sprouts)
            {
                if(node->GetDistance(sprout->rGetLocation()) < grid_spacing)
                {
                    too_close = true;
                }
            }
            if(too_close)
            {
                continue;
            }
        }

        // Get the grid index of the node
        unsigned grid_index = this->mpGridCalculator->GetGrid()->GetNearestCellIndex(node->rGetLocation());

        QLength cell_length1 = (node->GetSegment(0)->GetCellularProperties()->GetAverageCellLengthLongitudinal() +
                node->GetSegment(1)->GetCellularProperties()->GetAverageCellLengthLongitudinal())/2.0;
        QLength cell_length2 = (node->GetSegment(0)->GetCellularProperties()->GetAverageCellLengthCircumferential() +
                node->GetSegment(1)->GetCellularProperties()->GetAverageCellLengthCircumferential())/2.0;
        QArea cell_area = cell_length1*cell_length2;
        QLength segment_length = (node->GetSegment(0)->GetLength() + node->GetSegment(1)->GetLength())/2.0;
        QLength segment_radius = (node->GetSegment(0)->GetRadius() + node->GetSegment(1)->GetRadius())/2.0;
        double num_cells = std::round(2.0*M_PI*segment_radius*segment_length/cell_area);

        QConcentration vegf_conc = this->mVegfField[grid_index];
        double vegf_fraction = vegf_conc/(vegf_conc + mHalfMaxVegf);
        QTime time_step = SimulationTime::Instance()->GetTimeStep()*reference_time;
        double prob_tip_selection = this->mSproutingProbabilityPerCell*num_cells*time_step*vegf_fraction;
        if (RandomNumberGenerator::Instance()->ranf() < prob_tip_selection)
        {
            sprouts.push_back(node);
        }
    }
    return sprouts;
}

// Explicit instantiation
template class Owen2011SproutingRule<2>;
template class Owen2011SproutingRule<3>;
