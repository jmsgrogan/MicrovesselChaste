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

#include "SimulationTime.hpp"
#include "BaseUnits.hpp"
#include "TumourInsertionModifier.hpp"
#include "CancerCellMutationState.hpp"
#include "Part.hpp"
#include "Polygon.hpp"

template<unsigned DIM>
TumourInsertionModifier<DIM>::TumourInsertionModifier()
    : AbstractCellBasedSimulationModifier<DIM>(),
      mInsertionTime(24.0*3600.0*unit::seconds),
      mInsertionRadius(200.0e-6*unit::metres),
      mInsertionOrigin(DimensionalChastePoint<DIM>(0.0, 0.0, 0.0)),
      mTumourInserted(false)
{
}

template<unsigned DIM>
TumourInsertionModifier<DIM>::~TumourInsertionModifier()
{
}

template<unsigned DIM>
void TumourInsertionModifier<DIM>::SetInsertionRadius(units::quantity<unit::length> insertionRadius)
{
    mInsertionRadius = insertionRadius;
}

template<unsigned DIM>
void TumourInsertionModifier<DIM>::SetInsertionOrigin(DimensionalChastePoint<DIM> insertionOrigin)
{
    mInsertionOrigin = insertionOrigin;
}

template<unsigned DIM>
void TumourInsertionModifier<DIM>::SetInsertionTime(units::quantity<unit::time> insertionTime)
{
    mInsertionTime = insertionTime;
}

template<unsigned DIM>
void TumourInsertionModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    UpdateCellData(rCellPopulation);

    double current_time = SimulationTime::Instance()->GetTime();
    if(current_time*BaseUnits::Instance()->GetReferenceTimeScale()>=mInsertionTime and !mTumourInserted)
    {
        units::quantity<unit::length> reference_length = BaseUnits::Instance()->GetReferenceLengthScale();
        boost::shared_ptr<CancerCellMutationState> p_mutation_state =
                boost::shared_ptr<CancerCellMutationState>(new CancerCellMutationState);

        if(DIM==2)
        {
            std::shared_ptr<Part<DIM> > p_sub_domain = Part<DIM>::Create();
            std::shared_ptr<Polygon<DIM> > circle = p_sub_domain->AddCircle(mInsertionRadius, mInsertionOrigin);
            for (unsigned ind = 0; ind < rCellPopulation.rGetMesh().GetNumNodes(); ind++)
            {
                if (p_sub_domain->IsPointInPart(DimensionalChastePoint<DIM>(rCellPopulation.rGetMesh().GetNode(ind)->rGetLocation(),
                        reference_length)))
                {
                    if(rCellPopulation.IsCellAttachedToLocationIndex(ind))
                    {
                        rCellPopulation.GetCellUsingLocationIndex(ind)->SetMutationState(p_mutation_state);
                    }
                }
            }
        }
        mTumourInserted=true;
    }
}

template<unsigned DIM>
void TumourInsertionModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    /*
     * We must update CellData in SetupSolve(), otherwise it will not have been
     * fully initialised by the time we enter the main time loop.
     */
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void TumourInsertionModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Make sure the cell population is updated
    rCellPopulation.Update();
}

template<unsigned DIM>
void TumourInsertionModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class TumourInsertionModifier<1>;
template class TumourInsertionModifier<2>;
template class TumourInsertionModifier<3>;

