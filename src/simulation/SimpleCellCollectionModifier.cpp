/*

Copyright (c) 2005-2015, University of Oxford.
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

#include "MeshBasedCellPopulation.hpp"
#include "SimpleCellPopulation.hpp"
#include "boost/lexical_cast.hpp"
#include "SimpleCellCollectionModifier.hpp"
#include "SimpleCell.hpp"

template<unsigned DIM>
SimpleCellCollectionModifier<DIM>::SimpleCellCollectionModifier()
    : mPopulations()
{
}

template<unsigned DIM>
SimpleCellCollectionModifier<DIM>::~SimpleCellCollectionModifier()
{
}

template<unsigned DIM>
std::vector<boost::shared_ptr<SimpleCellPopulation<DIM> > > SimpleCellCollectionModifier<DIM>::GetSimpleCellPopulations()
{
    return mPopulations;
}

template<unsigned DIM>
void SimpleCellCollectionModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    boost::shared_ptr<SimpleCellPopulation<DIM> > p_population = SimpleCellPopulation<DIM>::Create();
    for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
         cell_iter != rCellPopulation.End(); ++cell_iter)
    {
        boost::shared_ptr<SimpleCell<DIM> > p_simple_cell = SimpleCell<DIM>::Create(rCellPopulation.GetLocationOfCellCentre(*cell_iter));
        p_population->AddCell(p_simple_cell);
    }
    mPopulations.push_back(p_population);

    // Update the cell data
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void SimpleCellCollectionModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    // Update the cell data
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void SimpleCellCollectionModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Make sure the cell population is updated
    rCellPopulation.Update();
}

template<unsigned DIM>
void SimpleCellCollectionModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
//template class AngiogenesisModifier<1>;
template class SimpleCellCollectionModifier<2>;
template class SimpleCellCollectionModifier<3>;

// Serialization for Boost >= 1.36
#include "SerializationExportWrapperForCpp.hpp"
EXPORT_TEMPLATE_CLASS1(SimpleCellCollectionModifier, 2)
EXPORT_TEMPLATE_CLASS1(SimpleCellCollectionModifier, 3)
