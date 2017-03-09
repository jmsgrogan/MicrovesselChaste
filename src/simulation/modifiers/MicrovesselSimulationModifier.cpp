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

#include "AbstractCellPopulation.hpp"
#include "boost/lexical_cast.hpp"
#include "MicrovesselSimulationModifier.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
MicrovesselSimulationModifier<DIM>::MicrovesselSimulationModifier()
    : AbstractCellBasedSimulationModifier<DIM>(),
      mpSolver(),
      mUpdateLabels(),
      mCellPopulationReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
      mCellPopulationReferenceConcentration(BaseUnits::Instance()->GetReferenceConcentrationScale())
{
}

template<unsigned DIM>
MicrovesselSimulationModifier<DIM>::~MicrovesselSimulationModifier()
{
}

template <unsigned DIM>
boost::shared_ptr<MicrovesselSimulationModifier<DIM> > MicrovesselSimulationModifier<DIM>::Create()
{
    MAKE_PTR(MicrovesselSimulationModifier<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
void MicrovesselSimulationModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    // Do set-up on the vascular tumour modifier
    if(mpSolver)
    {
        mpSolver->SetupFromModifier(rCellPopulation, mCellPopulationReferenceLength, mCellPopulationReferenceConcentration, outputDirectory);

        // Do the first solver increment
        mpSolver->Increment();
    }
    else
    {
        EXCEPTION("A MicrovesselSolver is required for this modifier.");
    }

    rCellPopulation.Update();

    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
         cell_iter != rCellPopulation.End();
         ++cell_iter)
    {
        // Store the cell's volume in CellData
        cell_iter->GetCellData()->SetItem("oxygen", 0.0);
    }

    // Update the cell data
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void MicrovesselSimulationModifier<DIM>::SetCellDataUpdateLabels(std::vector<std::string> labels)
{
    mUpdateLabels = labels;
}

template<unsigned DIM>
void MicrovesselSimulationModifier<DIM>::SetMicrovesselSolver(boost::shared_ptr<MicrovesselSolver<DIM> > pSolver)
{
    mpSolver = pSolver;
}

template<unsigned DIM>
void MicrovesselSimulationModifier<DIM>::SetCellPopulationLengthScale(units::quantity<unit::length> cellLengthScale)
{
    mCellPopulationReferenceLength = cellLengthScale;
}

template<unsigned DIM>
void MicrovesselSimulationModifier<DIM>::SetCellPopulationConcentrationScale(units::quantity<unit::concentration> cellConcentrationScale)
{
    mCellPopulationReferenceConcentration = cellConcentrationScale;
}

template<unsigned DIM>
void MicrovesselSimulationModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Increment the solver
    mpSolver->Increment();

    // Update the cell data
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void MicrovesselSimulationModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Make sure the cell population is updated
    rCellPopulation.Update();

    // Update the cell data
    mpSolver->UpdateCellData(mUpdateLabels);
}

template<unsigned DIM>
void MicrovesselSimulationModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class MicrovesselSimulationModifier<2>;
template class MicrovesselSimulationModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(MicrovesselSimulationModifier, 2)
EXPORT_TEMPLATE_CLASS1(MicrovesselSimulationModifier, 3)
