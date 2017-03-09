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



#include "GeometryTools.hpp"
#include "CellPopulationMigrationRule.hpp"
#include "RandomNumberGenerator.hpp"

template<unsigned DIM>
CellPopulationMigrationRule<DIM>::CellPopulationMigrationRule()
    : LatticeBasedMigrationRule<DIM>(),
      mVolumeFractionMap(),
      mPointCellMap()
{

}

template <unsigned DIM>
boost::shared_ptr<CellPopulationMigrationRule<DIM> > CellPopulationMigrationRule<DIM>::Create()
{
    MAKE_PTR(CellPopulationMigrationRule<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
CellPopulationMigrationRule<DIM>::~CellPopulationMigrationRule()
{

}

template<unsigned DIM>
void CellPopulationMigrationRule<DIM>::SetVolumeFraction(boost::shared_ptr<AbstractCellMutationState> mutation_state,
        double volume_fraction)
{
    if(volume_fraction >1.0)
    {
        EXCEPTION("Specified volume fractions should not be greater than 1.");
    }

    typedef std::map<boost::shared_ptr<AbstractCellMutationState> , double>::iterator it_type; it_type iterator;
    for(iterator = mVolumeFractionMap.begin(); iterator != mVolumeFractionMap.end(); iterator++)
    {
        if (iterator->first->IsSame(mutation_state))
        {
            iterator->second = volume_fraction;
            break;
        }
    }

    // if mutation state does not exist in map yet then add it to the map
    if (iterator == mVolumeFractionMap.end())
    {
        mVolumeFractionMap[mutation_state] = volume_fraction;
    }
}

template<unsigned DIM>
double CellPopulationMigrationRule<DIM>::GetOccupyingVolumeFraction(boost::shared_ptr<AbstractCellMutationState> mutation_state)
{
    typedef std::map<boost::shared_ptr<AbstractCellMutationState> , double>::iterator it_type; it_type iterator;
    for(iterator = mVolumeFractionMap.begin(); iterator != mVolumeFractionMap.end(); iterator++)
    {
        if (iterator->first->IsSame(mutation_state))
        {
            return iterator->second;
        }
    }

    // if a map is not provided or if the prescribed mutation state is not in the map then the
    // occupying volume fraction is 1.
    return 1;
}

template<unsigned DIM>
std::vector<double> CellPopulationMigrationRule<DIM>::GetNeighbourMovementProbabilities(boost::shared_ptr<VesselNode<DIM> > pNode,
                                                       std::vector<unsigned> neighbourIndices, unsigned gridIndex)
{
    std::vector<double> probability_of_moving(neighbourIndices.size(), 0.0);

    for(unsigned idx=0; idx<neighbourIndices.size(); idx++)
    {
        // Check for cell occupancy, if it is occupied don;t go anywhere
        double total_occupancy = 0.0;
        for(unsigned jdx=0; jdx<mPointCellMap[idx].size(); jdx++)
        {
            total_occupancy += GetOccupyingVolumeFraction(mPointCellMap[idx][jdx]->GetMutationState());
        }

        if(total_occupancy>=1.0)
        {
            continue;
        }

        // Make sure that tip cell does not try to move into a location already occupied by the vessel that it comes from
        DimensionalChastePoint<DIM> neighbour_location = this->mpGridCalculator->GetGrid()->GetLocationOfGlobalIndex(neighbourIndices[idx]);

        bool already_attached = false;
        for (unsigned seg_index = 0; seg_index < pNode->GetNumberOfSegments(); seg_index++)
        {
            if(pNode->GetSegment(seg_index)->GetOppositeNode(pNode)->IsCoincident(neighbour_location))
            {
                already_attached = true;
                break;
            }
        }

        // Also ensure that the new location would not try to cross a vessel which is oriented diagonally
        // TODO: Very slow, bottleneck
//        if(already_attached or this->mpVesselNetwork->VesselCrossesLineSegment(neighbour_location, pNode->rGetLocation()))
//        {
//            continue;
//        }

        if(already_attached)
        {
            continue;
        }

        // Simple rule, equal probability for all directions
        probability_of_moving[idx] = this->mMovementProbability * SimulationTime::Instance()->GetTimeStep();
    }
    return probability_of_moving;
}

template<unsigned DIM>
std::vector<int> CellPopulationMigrationRule<DIM>::GetIndices(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    if(!this->mpGridCalculator)
    {
        EXCEPTION("A regular grid is required for this type of migration rule.");
    }

    if(!this->mpVesselNetwork)
    {
        EXCEPTION("A vessel network is required for this type of migration rule.");
    }

    if(!this->mpCellPopulation)
    {
        EXCEPTION("A cell population is required for this type of migration rule.");
    }

    mPointCellMap = this->mpGridCalculator->rGetCellMap();

    // Set up the output indices vector
    std::vector<int> indices(rNodes.size(), -1);

    // Get the point-node map from the regular grid
    std::vector<std::vector<boost::shared_ptr<VesselNode<DIM> > > > point_node_map = this->mpGridCalculator->rGetVesselNodeMap();

    // Need a regular grid for this rule
    boost::shared_ptr<RegularGrid<DIM> > p_regular_grid =
            boost::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());

    if(!p_regular_grid)
    {
        EXCEPTION("Can't cast to regular grid");
    }

    // Get the neighbour data from the regular grid
    std::vector<std::vector<unsigned> > neighbour_indices = p_regular_grid->rGetNeighbourData();

    // Loop over all nodes, if they can move set the index
    for(unsigned idx = 0; idx < rNodes.size(); idx++)
    {
        // Get the grid index of the node
        unsigned grid_index = p_regular_grid->GetNearestLocationIndex(rNodes[idx]->rGetLocation());

        // Get the probability of moving into each of the neighbour sites
        std::vector<double> probability_of_moving = GetNeighbourMovementProbabilities(rNodes[idx], neighbour_indices[grid_index], grid_index);

        // Get the index of the neighbour to move into
        double sum = std::fabs(std::accumulate(probability_of_moving.begin(), probability_of_moving.end(), 0.0));
        if(sum > 0.0)
        {
            indices[idx] = LatticeBasedMigrationRule<DIM>::GetNeighbourMovementIndex(probability_of_moving, neighbour_indices[grid_index]);
        }
    }
    return indices;
}

// Explicit instantiation
template class CellPopulationMigrationRule<2> ;
template class CellPopulationMigrationRule<3> ;
