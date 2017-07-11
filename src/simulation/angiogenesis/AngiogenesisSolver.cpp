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

#include <boost/lexical_cast.hpp>
#include "UblasIncludes.hpp"
#include "RandomNumberGenerator.hpp"
#include "VesselNode.hpp"
#include "VesselSegment.hpp"
#include "VesselNode.hpp"
#include "AngiogenesisSolver.hpp"
#include "StalkCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "VesselNetworkWriter.hpp"
#include "BaseUnits.hpp"
#include "Timer.hpp"
#include "VesselNetworkGeometryCalculator.hpp"

#include "Debug.hpp"

template<unsigned DIM>
AngiogenesisSolver<DIM>::AngiogenesisSolver() :
        mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mpNetwork(),
        mNodeAnastamosisRadius(5_um),
        mpMigrationRule(),
        mpSproutingRule(),
        mpBoundingDomain(),
        mpFileHandler(),
        mpGridCalculator(),
        mpCellPopulation(),
        mCellPopulationReferenceLength(5_um),
        mTipCells(),
        mCellNodeMap(),
        mDoAnastamosis(true)
{

}

template<unsigned DIM>
AngiogenesisSolver<DIM>::~AngiogenesisSolver()
{

}

template<unsigned DIM>
std::shared_ptr<AngiogenesisSolver<DIM> > AngiogenesisSolver<DIM>::Create()
{
    return std::make_shared<AngiogenesisSolver<DIM> >();

}

template<unsigned DIM>
bool AngiogenesisSolver<DIM>::IsSproutingRuleSet()
{
    return bool(mpSproutingRule);
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetAnastamosisRadius(QLength radius)
{
    mNodeAnastamosisRadius = radius;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetBoundingDomain(PartPtr<DIM> pDomain)
{
    mpBoundingDomain = pDomain;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetDoAnastomosis(bool doAnastomosis)
{
    mDoAnastamosis = doAnastomosis;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetCellPopulation(std::shared_ptr<AbstractCellPopulation<DIM> > cell_population,
        QLength cellPopulationReferenceLength)
{
    mpCellPopulation = cell_population;
    mCellPopulationReferenceLength = cellPopulationReferenceLength;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetMigrationRule(std::shared_ptr<AbstractMigrationRule<DIM> > pMigrationRule)
{
    mpMigrationRule = pMigrationRule;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetOutputFileHandler(std::shared_ptr<OutputFileHandler> pHandler)
{
    mpFileHandler = pHandler;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetSproutingRule(std::shared_ptr<AbstractSproutingRule<DIM> > pSproutingRule)
{
    mpSproutingRule = pSproutingRule;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetVesselGridCalculator(std::shared_ptr<GridCalculator<DIM> > pVesselGrid)
{
    mpGridCalculator = pVesselGrid;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::DoSprouting()
{
    // Get the candidate sprouts and set them as migrating
    std::vector<VesselNodePtr<DIM> > candidate_sprouts = mpSproutingRule->GetSprouts(mpNetwork->GetNodes());
    for(auto& sprout:candidate_sprouts)
    {
        sprout->SetIsMigrating(true);
    }
    UpdateNodalPositions(true);
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::UpdateNodalPositions(bool sprouting)
{
    // Move any nodes marked as migrating, either new sprouts or tips
    std::vector<VesselNodePtr<DIM> > nodes = mpNetwork->GetNodes();
    std::vector<VesselNodePtr<DIM> > tips;
    for (unsigned idx = 0; idx < nodes.size(); idx++)
    {
        if (sprouting)
        {
            if (nodes[idx]->IsMigrating() && nodes[idx]->GetNumberOfSegments() == 2)
            {
                tips.push_back(nodes[idx]);
            }
        }
        else
        {
            if (nodes[idx]->IsMigrating() && nodes[idx]->GetNumberOfSegments() == 1)
            {
                tips.push_back(nodes[idx]);
            }
        }
    }

    // Do lattice or off lattice movement
    if (mpGridCalculator)
    {
        mpMigrationRule->SetIsSprouting(sprouting);

        // If we have a cell population update the cell-point map
        std::vector<int> indices = mpMigrationRule->GetIndices(tips);
        for (unsigned idx = 0; idx < tips.size(); idx++)
        {
            if(tips[idx]->GetFlowProperties()->IsInputNode() or tips[idx]->GetFlowProperties()->IsOutputNode())
            {
                tips[idx]->SetIsMigrating(false);
                continue;
            }

            if (indices[idx] >= 0 )
            {
                if (sprouting)
                {
                    mpNetwork->FormSprout(tips[idx], mpGridCalculator->GetGrid()->GetGlobalCellLocation(indices[idx]));
                    tips[idx]->SetIsMigrating(false);
                    mpNetwork->UpdateAll();
                }
                else
                {
                    VesselNodePtr<DIM> p_new_node = VesselNode<DIM>::Create(tips[idx]);
                    p_new_node->SetLocation(mpGridCalculator->GetGrid()->GetGlobalCellLocation(indices[idx]));
                    mpNetwork->ExtendVessel(tips[idx]->GetSegment(0)->GetVessel(), tips[idx], p_new_node);
                    tips[idx]->SetIsMigrating(false);
                    p_new_node->SetIsMigrating(true);
                    mpNetwork->UpdateAll();
                }
            }
            else
            {
                if (sprouting && tips[idx]->GetNumberOfSegments() == 2)
                {
                    tips[idx]->SetIsMigrating(false);
                }
            }
        }
    }
    else
    {
        if (mpBoundingDomain)
        {
            mpMigrationRule->SetBoundingDomain(mpBoundingDomain);
        }
        mpMigrationRule->SetIsSprouting(sprouting);
        std::vector<Vertex<DIM> > movement_vectors = mpMigrationRule->GetDirections(tips);
        vtkSmartPointer<vtkPoints> candidate_tip_locations = vtkSmartPointer<vtkPoints>::New();
        std::vector<bool> candidate_tips_inside_domain(tips.size(), true);
        for (unsigned idx = 0; idx < tips.size(); idx++)
        {
            c_vector<double, 3> loc = (tips[idx]->rGetLocation() + movement_vectors[idx]).Convert(mReferenceLength);
            candidate_tip_locations->InsertNextPoint(&loc[0]);
        }
        if (mpBoundingDomain)
        {
            candidate_tips_inside_domain = mpBoundingDomain->IsPointInPart(candidate_tip_locations);
        }
        for (unsigned idx = 0; idx < tips.size(); idx++)
        {
            if(tips[idx]->GetFlowProperties()->IsInputNode() or tips[idx]->GetFlowProperties()->IsOutputNode())
            {
                tips[idx]->SetIsMigrating(false);
                continue;
            }
            if (movement_vectors[idx].GetNorm2() > 0_m)
            {
                if (candidate_tips_inside_domain[idx])
                {
                    double loc[3];
                    candidate_tip_locations->GetPoint(idx, loc);
                    Vertex<DIM> dimensional_loc(loc, mReferenceLength);
                    if (sprouting)
                    {
                        mpNetwork->FormSprout(tips[idx], dimensional_loc);
                        tips[idx]->SetIsMigrating(false);
                    }
                    else
                    {
                        VesselNodePtr<DIM> p_new_node = VesselNode<DIM>::Create(tips[idx]);
                        p_new_node->SetLocation(dimensional_loc);
                        mpNetwork->ExtendVessel(tips[idx]->GetSegment(0)->GetVessel(), tips[idx], p_new_node);
                        tips[idx]->SetIsMigrating(false);
                        p_new_node->SetIsMigrating(true);
                    }
                }
            }
            else
            {
                if (sprouting && tips[idx]->GetNumberOfSegments() == 2)
                {
                    tips[idx]->SetIsMigrating(false);
                }
            }
        }
    }
    mpNetwork->UpdateAll();
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::DoAnastamosis()
{
    std::vector<VesselNodePtr<DIM> > nodes = mpNetwork->GetNodes();
    std::map<VesselNodePtr<DIM> , std::shared_ptr<VesselSegment<DIM> > > node_segment_map;
    std::map<std::shared_ptr<VesselSegment<DIM> > , std::vector<VesselNodePtr<DIM> > > segment_node_map;

    for (unsigned idx = 0; idx < nodes.size(); idx++)
    {
        // If this is currently a tip
        if (nodes[idx]->IsMigrating() && nodes[idx]->GetNumberOfSegments() == 1)
        {
            if (mpGridCalculator)
            {
                std::vector<std::vector<VesselNodePtr<DIM> > > point_node_map = mpGridCalculator->rGetVesselNodeMap();
                unsigned grid_index = mpGridCalculator->GetGrid()->GetNearestCellIndex(nodes[idx]->rGetLocation());

                if (point_node_map[grid_index].size() >= 2)
                {
                    VesselNodePtr<DIM> p_merge_node = VesselNode<DIM>::Create(nodes[idx]);
                    if (point_node_map[grid_index][0] == nodes[idx])
                    {
                        p_merge_node = mpNetwork->DivideVessel(
                                point_node_map[grid_index][1]->GetSegment(0)->GetVessel(), nodes[idx]->rGetLocation());
                    }
                    else
                    {
                        p_merge_node = mpNetwork->DivideVessel(
                                point_node_map[grid_index][0]->GetSegment(0)->GetVessel(), nodes[idx]->rGetLocation());
                    }

                    // Replace the tip node with the merge node
                    p_merge_node->SetIsMigrating(false);
                    if (nodes[idx]->GetSegment(0)->GetNode(0) == nodes[idx])
                    {
                        nodes[idx]->GetSegment(0)->ReplaceNode(0, p_merge_node);
                    }
                    else
                    {
                        nodes[idx]->GetSegment(0)->ReplaceNode(1, p_merge_node);
                    }
                    mpNetwork->UpdateAll();
                }
            }
            else
            {
                // Get the nearest segment and check if it is close enough to the node for a merge
                std::shared_ptr<VesselSegment<DIM> > p_nearest_segment;
                QLength distance = VesselNetworkGeometryCalculator<DIM>::GetNearestSegment(mpNetwork, nodes[idx],
                        p_nearest_segment, false, 3.0*mNodeAnastamosisRadius);
                if(p_nearest_segment)
                {
                    if (distance <= mNodeAnastamosisRadius && nodes[idx]->GetSegment(0)->GetLength() > distance)
                    {
                        // Populate the segment node map
                        node_segment_map[nodes[idx]] = p_nearest_segment;
                        segment_node_map[p_nearest_segment].push_back(nodes[idx]);
                    }
                }
            }
        }
    }

    typename std::map<VesselNodePtr<DIM> , std::shared_ptr<VesselSegment<DIM> > >::iterator node_iter;
    for(node_iter = node_segment_map.begin(); node_iter != node_segment_map.end(); node_iter++)
    {
        Vertex<DIM> original_location = (*node_iter).first->rGetLocation();
        if (mNodeAnastamosisRadius > 0.0 * unit::metres)
        {
            Vertex<DIM> divide_location = (*node_iter).second->GetPointProjection(
                    original_location, true);
            (*node_iter).first->SetLocation(divide_location);
        }

        std::vector<VesselNodePtr<DIM> > other_nodes = segment_node_map[(*node_iter).second];

        VesselNodePtr<DIM> p_merge_node = mpNetwork->DivideVessel(
                (*node_iter).second->GetVessel(), (*node_iter).first->rGetLocation());

        // If we have removed the segment and any other nodes need it, replace the segment in the map
        if(p_merge_node->GetNumberOfSegments()>1)
        {
            if(p_merge_node->GetSegment(0)!=(*node_iter).second and p_merge_node->GetSegment(1)!=(*node_iter).second)
            {
                for(unsigned idx=0;idx<other_nodes.size();idx++)
                {
                    if(p_merge_node->GetSegment(0)->GetDistance(other_nodes[idx]->rGetLocation())>
                    p_merge_node->GetSegment(1)->GetDistance(other_nodes[idx]->rGetLocation()))
                    {
                        node_segment_map[other_nodes[idx]] =  p_merge_node->GetSegment(1);
                        segment_node_map[p_merge_node->GetSegment(1)].push_back(other_nodes[idx]);
                    }
                    else
                    {
                        node_segment_map[other_nodes[idx]] =  p_merge_node->GetSegment(0);
                        segment_node_map[p_merge_node->GetSegment(0)].push_back(other_nodes[idx]);
                    }
                }
            }
        }

        // Replace the node at the end of the migrating tip with the merge node
        if (((*node_iter).first->GetSegment(0)->GetNode(0) == p_merge_node)
                || ((*node_iter).first->GetSegment(0)->GetNode(1) == p_merge_node))
        {
            (*node_iter).first->SetLocation(original_location);
        }
        else
        {
            p_merge_node->SetIsMigrating(false);
            if ((*node_iter).first->GetSegment(0)->GetNode(0) == (*node_iter).first)
            {
                (*node_iter).first->GetSegment(0)->ReplaceNode(0, p_merge_node);
            }
            else
            {
                (*node_iter).first->GetSegment(0)->ReplaceNode(1, p_merge_node);
            }
        }
        mpNetwork->Modified();
    }
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::Increment()
{
    if (!mpNetwork)
    {
        EXCEPTION("The angiogenesis solver needs an initial vessel network");
    }

    if (mpGridCalculator)
    {
        mpGridCalculator->SetVesselNetwork(mpNetwork);
        if (mpCellPopulation)
        {
            mpGridCalculator->SetCellPopulation(*mpCellPopulation, mCellPopulationReferenceLength,
                    BaseUnits::Instance()->GetReferenceConcentrationScale());
        }
    }

    if (mpMigrationRule)
    {
        mpMigrationRule->SetNetwork(mpNetwork);
        if (mpGridCalculator)
        {
            mpMigrationRule->SetGridCalculator(mpGridCalculator);
        }
        if (mpCellPopulation)
        {
            mpMigrationRule->SetCellPopulation(mpCellPopulation);
        }
    }

    if (mpSproutingRule)
    {
        mpSproutingRule->SetVesselNetwork(mpNetwork);
        if (mpGridCalculator)
        {
            mpSproutingRule->SetGridCalculator(mpGridCalculator);
        }
    }

    // Move any migrating nodes
    UpdateNodalPositions();

    // Check for anastamosis
    if(mDoAnastamosis)
    {
        DoAnastamosis();
    }

    // Do sprouting
    if (mpSproutingRule)
    {
        DoSprouting();
        if(mDoAnastamosis)
        {
            DoAnastamosis();
        }
    }
    // If there is a cell population, update it.
    if (mpCellPopulation)
    {
        // First label all existing Tip ECs as sprouts
        MAKE_PTR(StalkCellMutationState, p_EC_state);
        MAKE_PTR(TipCellMutationState, p_EC_Tip_state);
        CellPtr p_reference_cell;
        for (typename AbstractCellPopulation<DIM, DIM>::Iterator cell_iter = mpCellPopulation->Begin();
                cell_iter != mpCellPopulation->End(); ++cell_iter)
        {
            p_reference_cell = (*cell_iter);
            if ((*cell_iter)->GetMutationState()->IsSame(p_EC_Tip_state))
            {
                (*cell_iter)->SetMutationState(p_EC_state);
            }
        }

        // Then create new tip cells corresponding to vessel tips
        std::vector<VesselNodePtr<DIM> > nodes = mpNetwork->GetNodes();
        for (unsigned idx = 0; idx < nodes.size(); idx++)
        {
            if (nodes[idx]->IsMigrating())
            {
                unsigned location_index = mpGridCalculator->GetGrid()->GetNearestCellIndex(nodes[idx]->rGetLocation());

                // If there is already a stalk cell here it means a vessel tip has stayed in the same location, set it to tip type
                if(mpCellPopulation->IsCellAttachedToLocationIndex(location_index))
                {
                    if (mpCellPopulation->GetCellUsingLocationIndex(location_index)->GetMutationState()->IsSame(p_EC_state))
                    {
                        mpCellPopulation->GetCellUsingLocationIndex(location_index)->SetMutationState(p_EC_Tip_state);
                    }
                }
                else
                {
                    // Make a new cell here
                    CellPtr p_new_cell(new Cell(p_EC_Tip_state, p_reference_cell->GetCellCycleModel()->CreateCellCycleModel()));
                    p_new_cell->GetCellCycleModel()->InitialiseDaughterCell();
                    p_new_cell->SetApoptosisTime(p_reference_cell->GetApoptosisTime());

                    // Place them in the population
                    mpCellPopulation->AddCellUsingLocationIndex(location_index, p_new_cell); // this doesn't actually add a cell!
                    mpCellPopulation->rGetCells().push_back(p_new_cell); // do it manually here...
                }
            }
        }
    }
}

template<unsigned DIM>
void AngiogenesisSolver<DIM>::Run(bool writeOutput)
{
    // Set up a vessel network writer
    std::shared_ptr<VesselNetworkWriter<DIM> > p_network_writer = VesselNetworkWriter<DIM>::Create();

    // Loop for the duration of the simulation time
    while (!SimulationTime::Instance()->IsFinished())
    {
        // Write the vessel network if appropriate
        if (writeOutput && mpFileHandler && mpNetwork)
        {
            p_network_writer->SetFileName(
                    mpFileHandler->GetOutputDirectoryFullPath() + "/vessel_network_"
                            + boost::lexical_cast<std::string>(SimulationTime::Instance()->GetTimeStepsElapsed())
                            + ".vtp");
            p_network_writer->SetVesselNetwork(mpNetwork);
            p_network_writer->Write();
            if (mpCellPopulation)
            {
                mpCellPopulation->OpenWritersFiles(*mpFileHandler);
                mpCellPopulation->WriteResultsToFiles(mpFileHandler->GetRelativePath());
                mpCellPopulation->CloseWritersFiles();
            }
        }

        // Increment the solver and simulation time
        Increment();
        SimulationTime::Instance()->IncrementTimeOneStep();
    }
}

// Explicit instantiation
template class AngiogenesisSolver<2> ;
template class AngiogenesisSolver<3> ;
