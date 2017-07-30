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
#include "GeometryTools.hpp"
#include "UblasIncludes.hpp"
#include "UblasCustomFunctions.hpp"
#include "BaseUnits.hpp"
#include "Connor17Parameters.hpp"
#include "Owen11Parameters.hpp"
#include "OffLatticeSproutingRule.hpp"

template<unsigned DIM>
OffLatticeSproutingRule<DIM>::OffLatticeSproutingRule()
    : AbstractSproutingRule<DIM>(),
      mHalfMaxVegf(Connor17Parameters::mpVegfAtHalfReceptorOccupancy->GetValue("OffLatticeSproutingRule")),
      mVegfField()
{
    this->mSproutingProbabilityPerCell = Owen11Parameters::mpMaximumSproutingRate->GetValue("OffLatticeSproutingRule");
    this->mUseLateralInhibition = true;
    this->mUseVesselEndCutoff = true;
}

template<unsigned DIM>
OffLatticeSproutingRule<DIM>::~OffLatticeSproutingRule()
{

}

template <unsigned DIM>
void OffLatticeSproutingRule<DIM>::SetHalfMaxVegf(QConcentration halfMaxVegf)
{
    mHalfMaxVegf = halfMaxVegf;
}

template <unsigned DIM>
std::shared_ptr<OffLatticeSproutingRule<DIM> > OffLatticeSproutingRule<DIM>::Create()
{
    return std::make_shared<OffLatticeSproutingRule<DIM> >();

}

template<unsigned DIM>
std::vector<VesselNodePtr<DIM> > OffLatticeSproutingRule<DIM>::GetSprouts(const std::vector<VesselNodePtr<DIM> >& rNodes)
{
    if(!this->mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required for this type of sprouting rule.");
    }

    std::vector<QConcentration> probed_solutions(rNodes.size(), 0_M);
    vtkSmartPointer<vtkPoints> p_probe_locations = vtkSmartPointer<vtkPoints>::New();
    QLength reference_length = this->mpSolver->GetReferenceLength();

    if(this->mpSolver)
    {
        for(auto& node:rNodes)
        {
            c_vector<double, 3> loc = node->rGetLocation().Convert3(reference_length);
            p_probe_locations->InsertNextPoint(&loc[0]);
        }
        if(p_probe_locations->GetNumberOfPoints()>0)
        {
            probed_solutions = this->mpSolver->GetConcentrations(p_probe_locations);
        }
    }

    // Set up the output sprouts vector
    std::vector<VesselNodePtr<DIM> > sprouts;
    QTime reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    unsigned counter = 0;
    for(auto& node:rNodes)
    {
        // Only nodes with two segments can sprout
        if(node->GetNumberOfSegments() != 2)
        {
            continue;
        }

        // Check perfusion if needed
        if(this->mOnlySproutIfPerfused and node->GetFlowProperties()->GetPressure()==0_Pa)
        {
            continue;
        }

        // Apply a vessel end cutoff if needed
        QLength cell_length1 = (node->GetSegment(0)->GetCellularProperties()->GetAverageCellLengthLongitudinal() +
                node->GetSegment(1)->GetCellularProperties()->GetAverageCellLengthLongitudinal())/2.0;
        if(this->mUseVesselEndCutoff)
        {
            if(node->GetSegment(0)->GetVessel()->GetClosestEndNodeDistance(node->rGetLocation())< cell_length1)
            {
                continue;
            }
            if(node->GetSegment(1)->GetVessel()->GetClosestEndNodeDistance(node->rGetLocation())< cell_length1)
            {
                continue;
            }
        }

        // Check we are not too close to an existing candidate. This is different
        // from the vessel end cut-off as it relates to candidate tip cells.
        if(this->mUseLateralInhibition)
        {
            bool too_close = false;
            for(auto& sprout:sprouts)
            {
                // Any vessels same
                bool sv0_nv0_same = (sprout->GetSegment(0)->GetVessel() == node->GetSegment(0)->GetVessel());
                bool sv1_nv0_same = (sprout->GetSegment(1)->GetVessel() == node->GetSegment(0)->GetVessel());
                bool sv0_nv1_same = (sprout->GetSegment(0)->GetVessel() == node->GetSegment(1)->GetVessel());
                bool sv1_nv1_same = (sprout->GetSegment(1)->GetVessel() == node->GetSegment(1)->GetVessel());
                bool any_same = (sv0_nv0_same or sv1_nv0_same or sv0_nv1_same or sv1_nv1_same);
                if(any_same)
                {
                    if(node->GetDistance(sprout->rGetLocation()) < cell_length1)
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

        // If there is a vegf solution get P sprout
        QConcentration vegf_conc = 0_M;
        if(this->mpSolver)
        {
            vegf_conc = probed_solutions[counter];
        }

        QLength cell_length2 = (node->GetSegment(0)->GetCellularProperties()->GetAverageCellLengthCircumferential() +
                node->GetSegment(1)->GetCellularProperties()->GetAverageCellLengthCircumferential())/2.0;
        QArea cell_area = cell_length1*cell_length2;
        QLength segment_length = (node->GetSegment(0)->GetLength() + node->GetSegment(1)->GetLength())/2.0;
        QLength segment_radius = (node->GetSegment(0)->GetRadius() + node->GetSegment(1)->GetRadius())/2.0;
        double num_cells = std::round(2.0*M_PI*segment_radius*segment_length/cell_area);

        double vegf_fraction = vegf_conc/(vegf_conc + mHalfMaxVegf);
        QTime time_step = this->mSproutingProbabilityPerCell*SimulationTime::Instance()->GetTimeStep()*reference_time;
        double prob_per_time_step = this->mSproutingProbabilityPerCell*time_step*num_cells*vegf_fraction;
        if (RandomNumberGenerator::Instance()->ranf() < prob_per_time_step)
        {
            sprouts.push_back(node);
        }
        counter++;
    }
    return sprouts;
}

// Explicit instantiation
template class OffLatticeSproutingRule<2>;
template class OffLatticeSproutingRule<3>;
