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

#include "Owen11CellPopulationGenerator.hpp"
#include "BaseUnits.hpp"
#include "CancerCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "Cell.hpp"
#include "DefaultCellProliferativeType.hpp"
#include "Owen2011OxygenBasedCellCycleModel.hpp"
#include "VesselNetworkCellPopulationInteractor.hpp"
#include "CellLabelWriter.hpp"
#include "CellMutationStatesWriter.hpp"
#include "CellProliferativePhasesCountWriter.hpp"
#include "CellProliferativeTypesWriter.hpp"
#include "CellProliferativePhasesWriter.hpp"
#include "CellsGenerator.hpp"
#include "PottsMesh.hpp"
#include "Exception.hpp"

template<unsigned DIM>
Owen11CellPopulationGenerator<DIM>::Owen11CellPopulationGenerator() :
    mpVesselNetwork(),
    mpRegularGrid(),
    mpPottsMeshGenerator(),
    mpCancerCellMutationState(boost::shared_ptr<CancerCellMutationState>(new CancerCellMutationState)),
    mpNormalCellMutationState(),
    mpStalkCellMutationState(boost::shared_ptr<StalkCellMutationState>(new StalkCellMutationState)),
    mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
    mTumourRadius(0.0*unit::metres)
{

}

template<unsigned DIM>
boost::shared_ptr<Owen11CellPopulationGenerator<DIM> > Owen11CellPopulationGenerator<DIM>::Create()
{
    MAKE_PTR(Owen11CellPopulationGenerator<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
Owen11CellPopulationGenerator<DIM>::~Owen11CellPopulationGenerator()
{

}

template<unsigned DIM>
void Owen11CellPopulationGenerator<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpVesselNetwork = pNetwork;
}

template<unsigned DIM>
void Owen11CellPopulationGenerator<DIM>::SetRegularGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    mpRegularGrid = pGrid;
}

template<unsigned DIM>
void Owen11CellPopulationGenerator<DIM>::SetReferenceLengthScale(units::quantity<unit::length> lengthScale)
{
    mReferenceLength = lengthScale;
}

template<unsigned DIM>
void Owen11CellPopulationGenerator<DIM>::SetTumourRadius(units::quantity<unit::length> tumourRadius)
{
    mTumourRadius = tumourRadius;
}

template<unsigned DIM>
boost::shared_ptr<CaBasedCellPopulation<DIM> > Owen11CellPopulationGenerator<DIM>::Update()
{
    if(!mpRegularGrid)
    {
        EXCEPTION("A regular grid is required to set of the cell popaultion");
    }

    unsigned extents_z = 1;
    if(DIM==3)
    {
        extents_z = mpRegularGrid->GetExtents()[2];
    }

    mpPottsMeshGenerator = boost::shared_ptr<PottsMeshGenerator<DIM> >(new PottsMeshGenerator<DIM>(mpRegularGrid->GetExtents()[0], 0, 0,
                                                                                                   mpRegularGrid->GetExtents()[1], 0, 0,
                                                                                                   extents_z, 0, 0));

    PottsMesh<DIM>* p_mesh = mpPottsMeshGenerator->GetMesh();

    // There is a bug in Chaste causing index out of bounds for large grid spacing. It may be better to use a scaling here
    // so grid spacing is always one.
    p_mesh->Scale(mpRegularGrid->GetSpacing()/mReferenceLength, mpRegularGrid->GetSpacing()/mReferenceLength);
    std::vector<unsigned> location_indices;
    for (unsigned index=0; index < p_mesh->GetNumNodes(); index++)
    {
        location_indices.push_back(index);
    }

    // Fill the grid with cells
    std::vector<CellPtr> cells;
    MAKE_PTR(DefaultCellProliferativeType, p_diff_type);
    CellsGenerator<Owen2011OxygenBasedCellCycleModel, DIM> cells_generator;
    cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumNodes(), p_diff_type);
    boost::shared_ptr<CaBasedCellPopulation<DIM> > p_cell_population =
            boost::shared_ptr<CaBasedCellPopulation<DIM> >(new CaBasedCellPopulation<DIM> (*p_mesh, cells, location_indices));

    // Label stalk cells if the is a vessel network
    if(mpVesselNetwork)
    {
        VesselNetworkCellPopulationInteractor<DIM> interactor = VesselNetworkCellPopulationInteractor<DIM>();
        interactor.SetVesselNetwork(mpVesselNetwork);
        interactor.PartitionNetworkOverCells(*p_cell_population);
        interactor.LabelVesselsInCellPopulation(*p_cell_population, mpStalkCellMutationState, mpStalkCellMutationState);
    }

    // Set up tumour cells in the centre of the domain
    if(DIM==2)
    {
        DimensionalChastePoint<DIM> origin(double(mpRegularGrid->GetExtents()[0])*mpRegularGrid->GetSpacing()/(2.0*mReferenceLength),
                                         double(mpRegularGrid->GetExtents()[1])*mpRegularGrid->GetSpacing()/(2.0*mReferenceLength),
                                         0.0, mReferenceLength);
        boost::shared_ptr<Part<DIM> > p_sub_domain = Part<DIM>::Create();
        boost::shared_ptr<Polygon> circle = p_sub_domain->AddCircle(mTumourRadius, origin);
        for (unsigned ind = 0; ind < p_mesh->GetNumNodes(); ind++)
        {
            if (p_sub_domain->IsPointInPart(p_mesh->GetNode(ind)->rGetLocation()))
            {
                p_cell_population->GetCellUsingLocationIndex(ind)->SetMutationState(mpCancerCellMutationState);
            }
        }
    }
    else
    {
        DimensionalChastePoint<DIM> origin(double(mpRegularGrid->GetExtents()[0])*mpRegularGrid->GetSpacing()/(2.0*mReferenceLength),
                                         double(mpRegularGrid->GetExtents()[1])*mpRegularGrid->GetSpacing()/(2.0*mReferenceLength),
                                         double(mpRegularGrid->GetExtents()[2])*mpRegularGrid->GetSpacing()/(2.0*mReferenceLength),
                                         mReferenceLength);
        for(unsigned idx=0; idx<mpRegularGrid->GetNumberOfPoints(); idx++)
        {
            units::quantity<unit::length> distance = mpRegularGrid->GetLocationOf1dIndex(idx).GetDistance(origin);
            if(distance<=mTumourRadius)
            {
                p_cell_population->GetCellUsingLocationIndex(idx)->SetMutationState(mpCancerCellMutationState);
            }
        }
    }

    // Set up the cell cycle model . Note that Cell Based Chaste does not use dimensional analysis so we need to be careful with units.
    std::list<CellPtr> cells_updated = p_cell_population->rGetCells();
    std::list<CellPtr>::iterator it;
    for (it = cells_updated.begin(); it != cells_updated.end(); ++it)
    {
        (*it)->GetCellData()->SetItem("oxygen", 0.0);
        (*it)->GetCellData()->SetItem("VEGF", 0.0);
        (*it)->GetCellData()->SetItem("p53", 0.0);
        (*it)->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0.0);
        (*it)->GetCellData()->SetItem("Number_of_normal_neighbours", 0.0);
        (*it)->SetApoptosisTime(24); //hours
    }
    p_cell_population->SetOutputResultsForChasteVisualizer(false);
    p_cell_population->template AddCellWriter<CellLabelWriter>();
    p_cell_population->template AddCellWriter<CellMutationStatesWriter>();
    p_cell_population->template AddCellPopulationCountWriter<CellProliferativePhasesCountWriter>();
    p_cell_population->template AddCellWriter<CellProliferativeTypesWriter>();
    p_cell_population->template AddCellWriter<CellProliferativePhasesWriter>();

    return p_cell_population;
}

//template class Owen11CellPopulationGenerator<1> ;
template class Owen11CellPopulationGenerator<2> ;
template class Owen11CellPopulationGenerator<3> ;
