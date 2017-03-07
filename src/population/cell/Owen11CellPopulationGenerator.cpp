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
#include "Owen11CaUpdateRule.hpp"
#include "Owen11CaBasedDivisionRule.hpp"
#include "Owen11Parameters.hpp"
#include "Secomb04Parameters.hpp"
#include "GenericParameters.hpp"
#include "BaseUnits.hpp"
#include "RandomNumberGenerator.hpp"

template<unsigned DIM>
Owen11CellPopulationGenerator<DIM>::Owen11CellPopulationGenerator() :
    mpVesselNetwork(),
    mpGridCalculator(),
    mpPottsMeshGenerator(),
    mpCancerCellMutationState(boost::shared_ptr<CancerCellMutationState>(new CancerCellMutationState)),
    mpNormalCellMutationState(),
    mpStalkCellMutationState(boost::shared_ptr<StalkCellMutationState>(new StalkCellMutationState)),
    mReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
    mTumourRadius(0.0*unit::metres),
    mCellPopulationReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale())
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
void Owen11CellPopulationGenerator<DIM>::SetGridCalculator(boost::shared_ptr<GridCalculator<DIM> > pGrid)
{
    mpGridCalculator = pGrid;
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
    if(!mpGridCalculator)
    {
        EXCEPTION("A regular grid is required to set of the cell population");
    }

    boost::shared_ptr<RegularGrid<DIM > > p_grid =
            boost::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpGridCalculator->GetGrid());
    if(!p_grid)
    {
        EXCEPTION("Can't cast to regular grid during Setup");
    }

    c_vector<unsigned, 3> dimensions = p_grid->GetDimensions();
    unsigned extents_z = 1;
    if(DIM==3)
    {
        extents_z = dimensions[2];
    }

    mpPottsMeshGenerator = boost::shared_ptr<PottsMeshGenerator<DIM> >(new PottsMeshGenerator<DIM>(dimensions[0], 0, 0,
            dimensions[1], 0, 0,
                                                                                                   extents_z, 0, 0));

    PottsMesh<DIM>* p_mesh = mpPottsMeshGenerator->GetMesh();

    // There is a bug in Chaste causing index out of bounds for large grid spacing. It may be better to use a scaling here
    // so grid spacing is always one.

    units::quantity<unit::length> spacing = p_grid->GetSpacing();
    if(DIM==2)
    {
        p_mesh->Scale(spacing/mCellPopulationReferenceLength, spacing/mCellPopulationReferenceLength);
    }
    else
	{
        p_mesh->Scale(spacing/mCellPopulationReferenceLength, spacing/mCellPopulationReferenceLength,
                spacing/mCellPopulationReferenceLength);
	}

    std::vector<unsigned> location_indices;
    for (unsigned index=0; index < p_mesh->GetNumNodes(); index++)
    {
        location_indices.push_back(index);
    }

    // Fill the grid with cells
    std::vector<CellPtr> cells;
    MAKE_PTR(DefaultCellProliferativeType, p_diff_type);
    CellsGenerator<Owen2011OxygenBasedCellCycleModel, DIM> cells_generator;

//    MAKE_PTR(StemCellProliferativeType, p_stem_type);
//
//    double oxygen_concentration = 30.0;
//    for (unsigned i = 0; i < location_indices.size(); i++)
//    {
//        // Assign an oxygen based cell cycle model, which requires a dimension to be set.
//        Owen2011OxygenBasedCellCycleModel* const p_model = new Owen2011OxygenBasedCellCycleModel;
//        p_model->SetDimension(2);
//        CellPtr p_cell(new Cell(p_state, p_model));
//        p_cell->SetCellProliferativeType(p_stem_type);
//        p_cell->SetApoptosisTime(30);
//        cells.push_back(p_cell);
//        p_cell->GetCellData()->SetItem("oxygen", oxygen_concentration);
//    }

    //            // Create a tumour cells in a cylinder in the middle of the domain
    //            boost::shared_ptr<Part<3> > p_tumour_cell_region = GetInitialTumourCellRegion();
    //            std::vector<unsigned> location_indices = p_tumour_cell_region->GetContainingGridIndices(num_x, num_y, num_z, spacing);
    //
    //            std::vector<Node<3>*> nodes;
    //            for(unsigned idx=0; idx<location_indices.size(); idx++)
    //            {
    //                c_vector<double, 3> location = Grid::GetLocationOf1dIndex(location_indices[idx], num_x, num_y, spacing);
    //                nodes.push_back(new Node<3>(idx, location, false));
    //            }
    //            NodesOnlyMesh<3> mesh;
    //            mesh.ConstructNodesWithoutMesh(nodes, 1.5 * spacing);
    //            std::vector<CellPtr> cells;
    //            CellsGenerator<SimpleOxygenBasedCellCycleModel, 3> cells_generator;
    //            cells_generator.GenerateBasic(cells, mesh.GetNumNodes());
    //            NodeBasedCellPopulation<3> cell_population(mesh, cells);
    //            cell_population.SetAbsoluteMovementThreshold(2.0 * spacing);
    //            cell_population.AddCellWriter<CellLabelWriter>();

    cells_generator.GenerateBasicRandom(cells, p_mesh->GetNumNodes(), p_diff_type);
    boost::shared_ptr<CaBasedCellPopulation<DIM> > p_cell_population =
            boost::shared_ptr<CaBasedCellPopulation<DIM> >(new CaBasedCellPopulation<DIM> (*p_mesh, cells, location_indices, 2));

    // Label stalk cells if the is a vessel network
    if(mpVesselNetwork)
    {
        VesselNetworkCellPopulationInteractor<DIM> interactor = VesselNetworkCellPopulationInteractor<DIM>();
        interactor.SetVesselNetwork(mpVesselNetwork);
        interactor.PartitionNetworkOverCells(*p_cell_population, mCellPopulationReferenceLength);
//        interactor.LabelVesselsInCellPopulation(*p_cell_population, mpStalkCellMutationState, mpStalkCellMutationState);
    }

    // Set up tumour cells in the centre of the domain
    if(DIM==2)
    {
        DimensionalChastePoint<DIM> origin(double(dimensions[0])*spacing/(2.0*mReferenceLength),
                                         double(dimensions[1])*spacing/(2.0*mReferenceLength),
                                         0.0, mReferenceLength);
        boost::shared_ptr<Part<DIM> > p_sub_domain = Part<DIM>::Create();
        boost::shared_ptr<Polygon<DIM> > circle = p_sub_domain->AddCircle(mTumourRadius, origin);
        for (unsigned ind = 0; ind < p_mesh->GetNumNodes(); ind++)
        {
            if (p_sub_domain->IsPointInPart(DimensionalChastePoint<DIM>(p_mesh->GetNode(ind)->rGetLocation(), mCellPopulationReferenceLength)))
            {
                p_cell_population->GetCellUsingLocationIndex(ind)->SetMutationState(mpCancerCellMutationState);
            }
        }
    }
    else
    {
        double dimensionless_spacing = spacing/mReferenceLength;
        c_vector<double, DIM> dimensionless_origin = p_grid->GetOrigin().GetLocation(mReferenceLength);

        DimensionalChastePoint<DIM> origin(double(dimensions[0])*dimensionless_spacing/2.0 + dimensionless_origin[0],
                                         double(dimensions[1])*dimensionless_spacing/2.0 + dimensionless_origin[0],
                                         double(dimensions[2])*dimensionless_spacing/2.0 + dimensionless_origin[0],
										 mReferenceLength);

        for(unsigned idx=0; idx<p_grid->GetNumberOfLocations(); idx++)
        {
            units::quantity<unit::length> distance = p_grid->GetLocation(idx).GetDistance(origin);
            if(distance<=mTumourRadius)
            {
                p_cell_population->GetCellUsingLocationIndex(idx)->SetMutationState(mpCancerCellMutationState);
            }
        }
    }

    // Set up the cell cycle model . Note that Cell Based Chaste does not use dimensional analysis so we need to be careful with units.
    units::quantity<unit::pressure> initial_oxygen_tension(30.0*unit::mmHg);
    units::quantity<unit::solubility>  solubility = Secomb04Parameters::mpOxygenVolumetricSolubility->GetValue("Owen11CellPopulationGenerator") *
            GenericParameters::mpGasConcentrationAtStp->GetValue("Owen11CellPopulationGenerator");
    units::quantity<unit::concentration>  initial_oxygen_concentration = initial_oxygen_tension*solubility;

    std::list<CellPtr> cells_updated = p_cell_population->rGetCells();
    std::list<CellPtr>::iterator it;
    for (it = cells_updated.begin(); it != cells_updated.end(); ++it)
    {
        (*it)->GetCellData()->SetItem("Phi", 0.99*RandomNumberGenerator::Instance()->ranf());
        (*it)->GetCellData()->SetItem("oxygen", initial_oxygen_concentration/BaseUnits::Instance()->GetReferenceConcentrationScale());
        (*it)->GetCellData()->SetItem("VEGF", 0.0);
        (*it)->GetCellData()->SetItem("p53", 0.0);
        (*it)->GetCellData()->SetItem("Number_of_cancerous_neighbours", 0.0);
        (*it)->GetCellData()->SetItem("Number_of_normal_neighbours", 0.0);
//        (*it)->SetApoptosisTime(24); //hours
    }

    // Specify an update rule
    MAKE_PTR(Owen11CaUpdateRule<DIM>, p_update_rule);
    p_update_rule->SetVesselNetwork(mpVesselNetwork);
    p_update_rule->SetGridCalculator(mpGridCalculator);

    MAKE_PTR(Owen11CaBasedDivisionRule<DIM>, p_division_rule);
    p_division_rule->SetVesselNetwork(mpVesselNetwork);
    p_division_rule->SetGridCalculator(mpGridCalculator);

    p_cell_population->RemoveAllUpdateRules();
    p_cell_population->AddUpdateRule(p_update_rule);
    p_cell_population->SetCaBasedDivisionRule(p_division_rule);

    return p_cell_population;
}

//template class Owen11CellPopulationGenerator<1> ;
template class Owen11CellPopulationGenerator<2> ;
template class Owen11CellPopulationGenerator<3> ;
