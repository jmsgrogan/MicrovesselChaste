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

#include "Owen2011TrackingModifier.hpp"
#include "Owen2011OxygenBasedCellCycleModel.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "WildTypeCellMutationState.hpp"
#include "PottsMesh.hpp"

template<unsigned DIM>
Owen2011TrackingModifier<DIM>::Owen2011TrackingModifier()
: AbstractCellBasedSimulationModifier<DIM>()
  {
  }

template<unsigned DIM>
Owen2011TrackingModifier<DIM>::~Owen2011TrackingModifier()
{
}

template<unsigned DIM>
void Owen2011TrackingModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void Owen2011TrackingModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    /*
     * We must update CellData in SetupSolve(), otherwise it will not have been
     * fully initialised by the time we enter the main time loop.
     */
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void Owen2011TrackingModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{

    // Make sure the cell population is updated
    rCellPopulation.Update();

    // Recover each cell's concentrations from the ODEs and store in CellData
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
            cell_iter != rCellPopulation.End();
            ++cell_iter)
    {
        Owen2011OxygenBasedCellCycleModel* p_model = static_cast<Owen2011OxygenBasedCellCycleModel*>(cell_iter->GetCellCycleModel());
        double this_phi = p_model->GetPhi();
        double this_p53 = p_model->GetP53();
        double this_VEGF = p_model->GetVEGF();

        // Note that the state variables must be in the same order as listed in Owen2011OxygenBasedOdeSystem
        cell_iter->GetCellData()->SetItem("Phi", this_phi);
        cell_iter->GetCellData()->SetItem("p53", this_p53);
        cell_iter->GetCellData()->SetItem("VEGF", this_VEGF);

        // also calculate number of normal and cancerous neighbours
        unsigned number_of_normal_neighbours = 0;
        unsigned number_of_cancerous_neighbours = 0;

        std::set<unsigned> neighbouring_node_indices = static_cast<PottsMesh<DIM>& >(rCellPopulation.rGetMesh()).GetMooreNeighbouringNodeIndices(rCellPopulation.GetLocationIndexUsingCell(*cell_iter));
        std::set<unsigned>::iterator node_it;
        for (node_it = neighbouring_node_indices.begin(); node_it != neighbouring_node_indices.end(); node_it++)
        {
            if (rCellPopulation.IsCellAttachedToLocationIndex(*node_it))
            {
                std::set<CellPtr> neighbour_cells = rCellPopulation.GetCellsUsingLocationIndex(*node_it);
                std::set<CellPtr>::iterator cell_neighbour_it;
                for (cell_neighbour_it = neighbour_cells.begin(); cell_neighbour_it != neighbour_cells.end(); cell_neighbour_it++)
                {

                    if ((*cell_neighbour_it)->GetMutationState()->template IsType<WildTypeCellMutationState>())
                    {
                        number_of_normal_neighbours++;
                    }
                    if ((*cell_neighbour_it)->GetMutationState()->template IsType<CancerCellMutationState>() ||
                            (*cell_neighbour_it)->GetMutationState()->template IsType<QuiescentCancerCellMutationState>())
                    {
                        number_of_cancerous_neighbours++;
                    }

                }
            }
        }

        cell_iter->GetCellData()->SetItem("Number_of_cancerous_neighbours", number_of_cancerous_neighbours);
        cell_iter->GetCellData()->SetItem("Number_of_normal_neighbours", number_of_normal_neighbours);
    }
}

template<unsigned DIM>
void Owen2011TrackingModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class Owen2011TrackingModifier<1>;
template class Owen2011TrackingModifier<2>;
template class Owen2011TrackingModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(Owen2011TrackingModifier)
