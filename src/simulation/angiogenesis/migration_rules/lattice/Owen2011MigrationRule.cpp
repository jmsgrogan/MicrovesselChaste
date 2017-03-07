/*

Copyright (c) 2005-2016, University of Oxford.
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
#include "Owen2011MigrationRule.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"
#include "BaseUnits.hpp"
#include "Owen11Parameters.hpp"

template<unsigned DIM>
Owen2011MigrationRule<DIM>::Owen2011MigrationRule()
    : LatticeBasedMigrationRule<DIM>(),
      mCellMotility(Owen11Parameters::mpCellMotilityEndothelial->GetValue("Owen2011MigrationRule")),
      mCellChemotacticParameter(0.05*Owen11Parameters::mpChemotacticSensitivity->GetValue("Owen2011MigrationRule")),
      mVegfField()
{

}

template <unsigned DIM>
boost::shared_ptr<Owen2011MigrationRule<DIM> > Owen2011MigrationRule<DIM>::Create()
{
    MAKE_PTR(Owen2011MigrationRule<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
Owen2011MigrationRule<DIM>::~Owen2011MigrationRule()
{

}

template<unsigned DIM>
void Owen2011MigrationRule<DIM>::SetCellChemotacticParameter(units::quantity<unit::diffusivity_per_concentration> cellChemotacticParameter)
{
    mCellChemotacticParameter = cellChemotacticParameter;
}

template<unsigned DIM>
void Owen2011MigrationRule<DIM>::SetCellMotilityParameter(units::quantity<unit::diffusivity> cellMotility)
{
    mCellMotility = cellMotility;
}

template<unsigned DIM>
std::vector<int> Owen2011MigrationRule<DIM>::GetIndices(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    if(!this->mpSolver)
    {
        EXCEPTION("A DiscreteContinuum solver is required for this type of sprouting rule.");
    }

    mVegfField = this->mpSolver->GetConcentrations(this->mpGridCalculator->GetGrid());

    // Use the base class for the rest
    return LatticeBasedMigrationRule<DIM>::GetIndices(rNodes);
}

template<unsigned DIM>
std::vector<double> Owen2011MigrationRule<DIM>::GetNeighbourMovementProbabilities(boost::shared_ptr<VesselNode<DIM> > pNode,
                                                       std::vector<unsigned> neighbourIndices, unsigned gridIndex)
{
    std::vector<double> probability_of_moving(neighbourIndices.size(), 0.0);
    for(unsigned jdx=0; jdx<neighbourIndices.size(); jdx++)
    {
        // make sure that tip cell does not try to move into a location already occupied by the vessel that it comes from
        // i.e. that it doesn't loop back around
        DimensionalChastePoint<DIM> neighbour_location = this->mpGridCalculator->GetGrid()->GetLocationOfGlobalIndex(neighbourIndices[jdx]);

        bool sprout_already_attached_to_vessel_at_location = false;

        for (unsigned seg_index = 0; seg_index < pNode->GetNumberOfSegments(); seg_index++)
        {
            if(pNode->GetSegment(seg_index)->GetOppositeNode(pNode)->IsCoincident(neighbour_location))
            {
                 sprout_already_attached_to_vessel_at_location = true;
                 break;
            }
        }

        //ensure that the new sprout would not try to cross a vessel which is oriented diagonally.
        // todo bottleneck, dont think it is even needed for current neighbourhood
//        bool vessel_crosses_line_segment = this->mpVesselNetwork->VesselCrossesLineSegment(neighbour_location, pNode->rGetLocation());
        bool vessel_crosses_line_segment = false;

        if (!vessel_crosses_line_segment && !sprout_already_attached_to_vessel_at_location)
        {
            units::quantity<unit::concentration> VEGF_diff = mVegfField[neighbourIndices[jdx]] - mVegfField[gridIndex];
            units::quantity<unit::time> dt = SimulationTime::Instance()->GetTimeStep() * BaseUnits::Instance()->GetReferenceTimeScale();
            units::quantity<unit::length> dij = pNode->rGetLocation().GetDistance(neighbour_location);
            probability_of_moving[jdx] = ((mCellMotility * dt)/(2.0*dij*dij))*(1.0 + mCellChemotacticParameter*VEGF_diff/(2.0*mCellMotility));
            if (probability_of_moving[jdx] < 0.0)
            {
                probability_of_moving[jdx] = 0.0;
            }
        }
    }
    return probability_of_moving;
}

// Explicit instantiation
template class Owen2011MigrationRule<2> ;
template class Owen2011MigrationRule<3> ;
