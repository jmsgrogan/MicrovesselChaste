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

#include "OffLatticeSproutingRule.hpp"
#include "RandomNumberGenerator.hpp"
#include "GeometryTools.hpp"
#include "UblasIncludes.hpp"
#include "UblasCustomFunctions.hpp"
#include "BaseUnits.hpp"
#include "Owen11Parameters.hpp"

template<unsigned DIM>
OffLatticeSproutingRule<DIM>::OffLatticeSproutingRule()
    : AbstractSproutingRule<DIM>(),
      mTipExclusionRadius(80_um),
      mHalfMaxVegf(Owen11Parameters::mpVegfConventrationAtHalfMaxProbSprouting->GetValue("Owen2011SproutingRule")),
      mVegfField()
{
    // Owen11 equation is dimensionally inconsistent as it includes an extra vessel surface area term. In order to
    // have a similar magnitude multiply this value by a typical vessel surface area = 2*pi*R*L
    // = 2*pi*15*40
    this->mSproutingProbability = Owen11Parameters::mpMaximumSproutingRate->GetValue("Owen2011SproutingRule")*2.0*M_PI*15.0*40.0;
    this->mTipExclusionRadius = 80_um;
}

template<unsigned DIM>
OffLatticeSproutingRule<DIM>::~OffLatticeSproutingRule()
{

}

template <unsigned DIM>
std::shared_ptr<OffLatticeSproutingRule<DIM> > OffLatticeSproutingRule<DIM>::Create()
{
    return std::make_shared<OffLatticeSproutingRule<DIM> >();

}

template<unsigned DIM>
std::vector<std::shared_ptr<VesselNode<DIM> > > OffLatticeSproutingRule<DIM>::GetSprouts(const std::vector<std::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    if(!this->mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required for this type of sprouting rule.");
    }

    std::vector<QConcentration > probed_solutions(rNodes.size(), 0.0*unit::mole_per_metre_cubed);
    vtkSmartPointer<vtkPoints> p_probe_locations = vtkSmartPointer<vtkPoints>::New();
    QLength reference_length = this->mpSolver->GetReferenceLength();

    if(this->mpSolver)
    {
        for(unsigned idx=0; idx<rNodes.size(); idx++)
        {
            c_vector<double, 3> loc = rNodes[idx]->rGetLocation().Convert3(reference_length);
            p_probe_locations->InsertNextPoint(&loc[0]);
        }
        if(p_probe_locations->GetNumberOfPoints()>0)
        {
            probed_solutions = this->mpSolver->GetConcentrations(p_probe_locations);
        }
    }

    // Set up the output sprouts vector
    std::vector<VesselNodePtr<DIM> > sprouts;

    // Loop over all nodes and randomly select sprouts
    for(unsigned idx = 0; idx < rNodes.size(); idx++)
    {
        if(this->mOnlySproutIfPerfused)
        {
            if(rNodes[idx]->GetFlowProperties()->GetPressure()==0_Pa)
            {
                continue;
            }
        }

        // Only nodes with two segments can sprout
        if(rNodes[idx]->GetNumberOfSegments() != 2)
        {
            continue;
        }

        // Apply a vessel end cutoff if there is one
        if(this->mVesselEndCutoff > 0_m)
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
        if(mTipExclusionRadius>0_m)
        {
            bool too_close = false;
            for(unsigned jdx=0; jdx<sprouts.size(); jdx++)
            {
                // Any vessels same
                bool sv0_nv0_same = (sprouts[jdx]->GetSegment(0)->GetVessel() == rNodes[idx]->GetSegment(0)->GetVessel());
                bool sv1_nv0_same = (sprouts[jdx]->GetSegment(1)->GetVessel() == rNodes[idx]->GetSegment(0)->GetVessel());
                bool sv0_nv1_same = (sprouts[jdx]->GetSegment(0)->GetVessel() == rNodes[idx]->GetSegment(1)->GetVessel());
                bool sv1_nv1_same = (sprouts[jdx]->GetSegment(1)->GetVessel() == rNodes[idx]->GetSegment(1)->GetVessel());
                bool any_same = (sv0_nv0_same or sv1_nv0_same or sv0_nv1_same or sv1_nv1_same);
                if(any_same)
                {
                    if(rNodes[idx]->GetDistance(sprouts[jdx]->rGetLocation()) < mTipExclusionRadius)
                    {
                        too_close = true;
                    }
                }
            }
            if(too_close)
            {
                continue;
            }
        }
        QConcentration vegf_conc = 0.0*unit::mole_per_metre_cubed;
        if(this->mpSolver)
        {
            vegf_conc = probed_solutions[idx];
        }

        QLength reference_length_for_sprouting = 40_um;
        QLength node_length = (rNodes[idx]->GetSegment(0)->GetLength()+ rNodes[idx]->GetSegment(1)->GetLength())/2.0;
        double length_factor = node_length/reference_length_for_sprouting;

        double vegf_fraction = vegf_conc/(vegf_conc + mHalfMaxVegf);
        double max_prob_per_time_step = this->mSproutingProbability*SimulationTime::Instance()->GetTimeStep()*
                BaseUnits::Instance()->GetReferenceTimeScale()*length_factor;
        double prob_tip_selection = max_prob_per_time_step*vegf_fraction;
        if (RandomNumberGenerator::Instance()->ranf() < prob_tip_selection)
        {
            sprouts.push_back(rNodes[idx]);
        }
    }
    return sprouts;
}

template <unsigned DIM>
void OffLatticeSproutingRule<DIM>::SetTipExclusionRadius(QLength exclusionRadius)
{
    this->mTipExclusionRadius = exclusionRadius;
}

// Explicit instantiation
template class OffLatticeSproutingRule<2> ;
template class OffLatticeSproutingRule<3> ;
