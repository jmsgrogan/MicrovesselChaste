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
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"
#include "BaseUnits.hpp"
#include "Owen11Parameters.hpp"
#include "TipAttractionLatticeBasedMigrationRule.hpp"
#include "Connor17Parameters.hpp"

template<unsigned DIM>
TipAttractionLatticeBasedMigrationRule<DIM>::TipAttractionLatticeBasedMigrationRule()
    : LatticeBasedMigrationRule<DIM>(),
      mCellMotility(Connor17Parameters::mpMotilityCoefficient->GetValue("TipAttractionLatticeBasedMigrationRule")),
      mCellChemotacticParameter(Connor17Parameters::mpChemotacticCoefficient->GetValue("TipAttractionLatticeBasedMigrationRule")),
      mVegfField(),
      mTipAttractionRadius(Connor17Parameters::mpFilopodiaProbeDistance->GetValue("TipAttractionLatticeBasedMigrationRule")),
      mTipAttractionAngle(Connor17Parameters::mpFilopodiaSensingAngle->GetValue("TipAttractionLatticeBasedMigrationRule")),
      mTipSproutAttractionStrength(Connor17Parameters::mpTipVesselAttractionStrength->GetValue("TipAttractionLatticeBasedMigrationRule")),
      mTipTipAttractionStrength(Connor17Parameters::mpTipTipAttractionStrength->GetValue("TipAttractionLatticeBasedMigrationRule")),
      mUseTipAttraction(true)
{

}

template <unsigned DIM>
boost::shared_ptr<TipAttractionLatticeBasedMigrationRule<DIM> > TipAttractionLatticeBasedMigrationRule<DIM>::Create()
{
    MAKE_PTR(TipAttractionLatticeBasedMigrationRule<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
TipAttractionLatticeBasedMigrationRule<DIM>::~TipAttractionLatticeBasedMigrationRule()
{

}

template<unsigned DIM>
void TipAttractionLatticeBasedMigrationRule<DIM>::SetCellChemotacticParameter(units::quantity<unit::diffusivity_per_concentration> cellChemotacticParameter)
{
    mCellChemotacticParameter = cellChemotacticParameter;
}

template<unsigned DIM>
void TipAttractionLatticeBasedMigrationRule<DIM>::SetCellMotilityParameter(units::quantity<unit::diffusivity> cellMotility)
{
    mCellMotility = cellMotility;
}


template<unsigned DIM>
void TipAttractionLatticeBasedMigrationRule<DIM>::SetUseTipAttraction(bool useTipAttraction)
{
    mUseTipAttraction = useTipAttraction;
}

template<unsigned DIM>
std::vector<int> TipAttractionLatticeBasedMigrationRule<DIM>::GetIndices(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
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
std::vector<double> TipAttractionLatticeBasedMigrationRule<DIM>::GetNeighbourMovementProbabilities(boost::shared_ptr<VesselNode<DIM> > pNode,
                                                       std::vector<unsigned> neighbourIndices, unsigned gridIndex)
{
    std::vector<double> probability_of_moving(neighbourIndices.size(), 0.0);

    // Determine if there is tip-tip or tip-sprout attraction
    bool tip_found = false;
    bool sprout_found = false;
    DimensionalChastePoint<DIM> tip_point;
    DimensionalChastePoint<DIM> sprout_point;
    units::quantity<unit::velocity> tip_attraction_strength = 0.0*unit::metres_per_second;
    units::quantity<unit::velocity> sprout_attraction_strength = 0.0*unit::metres_per_second;
    units::quantity<unit::length> closest_tip_distance = 1e6*unit::metres;
    units::quantity<unit::length> closest_sprout_distance = 1e6*unit::metres;
    units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();

    if(!this->mIsSprouting and mUseTipAttraction)
    {
        std::vector<boost::shared_ptr<VesselNode<DIM> > > nearby_nodes = this->mpVesselNetwork->GetNodesInSphere(
                pNode->rGetLocation(), mTipAttractionRadius);
        // get the current vector
        c_vector<double, DIM> unit_vector = pNode->rGetLocation().GetLocation(reference_length) -
                pNode->GetSegment(0)->GetOppositeNode(pNode)->rGetLocation().GetLocation(reference_length);
        unit_vector /= norm_2(unit_vector);
        for(unsigned idx=0; idx<nearby_nodes.size(); idx++)
        {
            c_vector<double, DIM> vector_node_nbr = nearby_nodes[idx]->rGetLocation().GetLocation(reference_length) -
                    pNode->rGetLocation().GetLocation(reference_length);
            vector_node_nbr/=norm_2(vector_node_nbr);
            double angle = std::acos(inner_prod(vector_node_nbr, unit_vector));

            if(std::abs(angle)>M_PI)
            {
                angle = 2.0*M_PI - std::abs(angle);
            }
            if(std::abs(angle)<mTipAttractionAngle+1.e-3)
            {
                units::quantity<unit::length> tip_nbr_distance = pNode->GetDistance(nearby_nodes[idx]->rGetLocation());
                if(nearby_nodes[idx]->IsMigrating())
                {
                    if(tip_nbr_distance<closest_tip_distance)
                    {
                        closest_tip_distance = tip_nbr_distance;
                        tip_found = true;
                        tip_attraction_strength = mTipTipAttractionStrength;
                        tip_point = nearby_nodes[idx]->rGetLocation();
                    }
                }
                else
                {
                    if(tip_nbr_distance<closest_sprout_distance)
                    {
                        closest_sprout_distance = tip_nbr_distance;
                        sprout_found = true;
                        sprout_attraction_strength = mTipSproutAttractionStrength*
                                (1.0-angle/mTipAttractionAngle)*(1.0-(tip_nbr_distance*tip_nbr_distance/(mTipAttractionRadius*mTipAttractionRadius)));
                        sprout_point = nearby_nodes[idx]->rGetLocation();
                    }
                }
            }
        }
    }

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
        bool vessel_crosses_line_segment = false;
        if(this->mUseMooreNeighbourhood)
        {
            vessel_crosses_line_segment = this->mpVesselNetwork->VesselCrossesLineSegment(neighbour_location, pNode->rGetLocation());
        }

        if (!vessel_crosses_line_segment && !sprout_already_attached_to_vessel_at_location)
        {
            units::quantity<unit::concentration> VEGF_diff = mVegfField[neighbourIndices[jdx]] - mVegfField[gridIndex];
            if(VEGF_diff<0.0*unit::mole_per_metre_cubed)
            {
                VEGF_diff = 0.0* unit::mole_per_metre_cubed;
            }
            double k = 1.0; // lattice multiplication factor
            if(DIM==2 and this->mUseMooreNeighbourhood)
            {
                k=2.0;
            }
            if(DIM==3 and this->mUseMooreNeighbourhood)
            {
                k = 26.0/6.0;
            }
            units::quantity<unit::time> dt = SimulationTime::Instance()->GetTimeStep() * BaseUnits::Instance()->GetReferenceTimeScale();
            units::quantity<unit::length> dij = pNode->rGetLocation().GetDistance(neighbour_location);
            probability_of_moving[jdx] = (dt/(k*dij*dij))*(mCellMotility + mCellChemotacticParameter*VEGF_diff);

            if(!this->mIsSprouting and mUseTipAttraction)
            {
                // Add tip contribution
                units::quantity<unit::velocity> nbr_tip_attraction_strength = 0.0*unit::metres_per_second;
                units::quantity<unit::velocity> nbr_sprout_attraction_strength = 0.0*unit::metres_per_second;

                double grid_angle = 45.0*M_PI/180.0;
                if(this->mUseMooreNeighbourhood)
                {
                    grid_angle = 22.5*M_PI/180.0;
                }

                if(tip_found)
                {
                    c_vector<double, DIM> vector_node_nbr = neighbour_location.GetLocation(reference_length)
                            - pNode->rGetLocation().GetLocation(reference_length);
                    vector_node_nbr/=norm_2(vector_node_nbr);

                    c_vector<double, DIM> vector_node_tip = tip_point.GetLocation(reference_length) -
                            pNode->rGetLocation().GetLocation(reference_length);
                    vector_node_tip/=norm_2(vector_node_tip);
                    double angle = std::acos(inner_prod(vector_node_nbr, vector_node_tip));
                    if(std::abs(angle)>M_PI)
                    {
                        angle = 2.0*M_PI - std::abs(angle);
                    }

                    if(std::abs(angle)<=grid_angle+1.e-3)
                    {
                        nbr_tip_attraction_strength = tip_attraction_strength;
                    }
                }
                if(sprout_found)
                {
                    c_vector<double, DIM> vector_node_nbr = neighbour_location.GetLocation(reference_length) -
                            pNode->rGetLocation().GetLocation(reference_length);
                    vector_node_nbr/=norm_2(vector_node_nbr);

                    c_vector<double, DIM> vector_node_sprout = sprout_point.GetLocation(reference_length) -
                            pNode->rGetLocation().GetLocation(reference_length);
                    vector_node_sprout/=norm_2(vector_node_sprout);
                    double angle = std::acos(inner_prod(vector_node_nbr, vector_node_sprout));
                    if(std::abs(angle)>M_PI)
                    {
                        angle = 2.0*M_PI - std::abs(angle);
                    }
                    if(std::abs(angle)<=grid_angle+1.e-3)
                    {
                        nbr_sprout_attraction_strength = sprout_attraction_strength;
                    }
                }

                if(sprout_found or tip_found)
                {
                    if(nbr_tip_attraction_strength>nbr_sprout_attraction_strength)
                    {
                        probability_of_moving[jdx] += (dt/dij)*nbr_tip_attraction_strength;
                    }
                    else
                    {
                        probability_of_moving[jdx] += (dt/dij)*nbr_sprout_attraction_strength;
                    }
                }
            }

            if (probability_of_moving[jdx] < 0.0)
            {
                probability_of_moving[jdx] = 0.0;
            }
        }
    }
    return probability_of_moving;
}

// Explicit instantiation
template class TipAttractionLatticeBasedMigrationRule<2> ;
template class TipAttractionLatticeBasedMigrationRule<3> ;
