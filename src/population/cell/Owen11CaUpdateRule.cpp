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

#include "Owen11CaUpdateRule.hpp"
#include "Owen11Parameters.hpp"
#include "BaseUnits.hpp"
#include "CancerCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"

template<unsigned DIM>
Owen11CaUpdateRule<DIM>::Owen11CaUpdateRule()
    : AbstractCaUpdateRule<DIM>(),
      mDiffusionParameter(Owen11Parameters::mpCellMotilityCancer->GetValue("Owen11CaUpdateRule")),
      mpVesselNetwork(),
      mpGridCalculator(),
      mReferenceLengthScale(BaseUnits::Instance()->GetReferenceLengthScale()),
      mCancerCellCarryingCapacity(2)

{
}

template<unsigned DIM>
Owen11CaUpdateRule<DIM>::~Owen11CaUpdateRule()
{
}

template<unsigned DIM>
double Owen11CaUpdateRule<DIM>::EvaluateProbability(unsigned currentNodeIndex,
                                                               unsigned targetNodeIndex,
                                                               CaBasedCellPopulation<DIM>& rCellPopulation,
                                                               double dt,
                                                               double deltaX,
                                                               CellPtr pCell)
{
   if(pCell->GetMutationState()->IsType<CancerCellMutationState>() || pCell->GetMutationState()->IsType<QuiescentCancerCellMutationState>())
   {
       // Check if the carrying capacity is sufficient
       unsigned num_cells_at_site = 0;

       if(mpVesselNetwork and DIM>1)
       {
           if(!mpGridCalculator)
           {
               EXCEPTION("A regular grid is required for determining vessel based lattice occupancy");
           }
           if(mpGridCalculator->IsSegmentAtLocation(targetNodeIndex, false))
           {
               num_cells_at_site = 1;
           }
       }

       num_cells_at_site += rCellPopulation.GetCellsUsingLocationIndex(targetNodeIndex).size();

       // Get the sprouting probability
       c_vector<double, DIM> node_index_location = rCellPopulation.GetNode(currentNodeIndex)->rGetLocation();
       c_vector<double, DIM> node_neighbour_location = rCellPopulation.GetNode(targetNodeIndex)->rGetLocation();
       units::quantity<unit::length> grid_distance = norm_2(rCellPopulation.rGetMesh().GetVectorFromAtoB(node_index_location,
                                                                                                         node_neighbour_location))*mReferenceLengthScale;

       units::quantity<unit::time> time_increment = dt*BaseUnits::Instance()->GetReferenceTimeScale();
       double carrying_capacity_factor = (double(mCancerCellCarryingCapacity)-double(num_cells_at_site))/double(mCancerCellCarryingCapacity);
       return mDiffusionParameter*time_increment*carrying_capacity_factor/(2.0* units::pow<2>(grid_distance));
   }
   else
   {
       return 0.0;
   }
}

/**
 * Specialization to allow used with 1D classes
 * @param currentNodeIndex the current index
 * @param targetNodeIndex the target index
 * @param rCellPopulation the cell population
 * @param dt the time increment
 * @param deltaX the grid spacing
 * @param pCell the cell
 * @return the movement probability
 */
template<>
double Owen11CaUpdateRule<1>::EvaluateProbability(unsigned currentNodeIndex,
                                                               unsigned targetNodeIndex,
                                                               CaBasedCellPopulation<1>& rCellPopulation,
                                                               double dt,
                                                               double deltaX,
                                                               CellPtr pCell)
{
    EXCEPTION("This update rule is not supported in 1D");
}

template<unsigned DIM>
void Owen11CaUpdateRule<DIM>::SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pVesselNetwork)
{
    mpVesselNetwork = pVesselNetwork;
}

template<unsigned DIM>
void Owen11CaUpdateRule<DIM>::SetReferenceLengthScale(units::quantity<unit::length> referenceLengthScale)
{
    mReferenceLengthScale = referenceLengthScale;
}

template<unsigned DIM>
void Owen11CaUpdateRule<DIM>::SetGridCalculator(std::shared_ptr<GridCalculator<DIM> > pRegularGrid)
{
    mpGridCalculator = pRegularGrid;
}

template<unsigned DIM>
units::quantity<unit::diffusivity> Owen11CaUpdateRule<DIM>::GetDiffusionParameter()
{
    return mDiffusionParameter;
}

template<unsigned DIM>
void Owen11CaUpdateRule<DIM>::SetDiffusionParameter(units::quantity<unit::diffusivity> diffusionParameter)
{
    mDiffusionParameter = diffusionParameter;
}

template<unsigned DIM>
void Owen11CaUpdateRule<DIM>::OutputUpdateRuleParameters(out_stream& rParamsFile)
{
    *rParamsFile << "\t\t\t<DiffusionParameter>" << mDiffusionParameter << "</DiffusionParameter>\n";

    // Call method on direct parent class
    AbstractCaUpdateRule<DIM>::OutputUpdateRuleParameters(rParamsFile);
}

// Explicit instantiation
template class Owen11CaUpdateRule<2u>;
template class Owen11CaUpdateRule<3u>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(Owen11CaUpdateRule)
