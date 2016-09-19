#include "CancerCellMutationState.hpp"
#include "SimpleCell.hpp"
#include "SimpleCellPopulation.hpp"
#include "Owen2011OxygenBasedCellCycleModel.hpp"

template class SimpleCell<3>;
template class SimpleCellPopulation<3>;
template class CaBasedCellPopulation<3>;

// Helper functions
boost::shared_ptr<Cell> GenerateCell(const std::string& mutation_state, const std::string& cell_cycle_model)
{
    MAKE_PTR(CancerCellMutationState, p_state); // Default state

    Owen2011OxygenBasedCellCycleModel* const p_model = new Owen2011OxygenBasedCellCycleModel;
    p_model->SetDimension(2);

    CellPtr p_cell(new Cell(p_state, p_model));
    p_cell->SetApoptosisTime(30.0);
    p_cell->GetCellData()->SetItem("oxygen", 30.0);

    MAKE_PTR(StemCellProliferativeType, p_stem_type); // Default Type
    p_cell->SetCellProliferativeType(p_stem_type);
    return p_cell;
}

boost::shared_ptr<CaBasedCellPopulation<3> > GenerateCaBasedCellPopulation(boost::shared_ptr<PottsMesh<3u> > pMesh, std::vector<CellPtr> cells, std::vector<unsigned> location_indices)
{
    boost::shared_ptr<CaBasedCellPopulation<3> > p_cell_population(new CaBasedCellPopulation<3>(*pMesh, cells, location_indices));
    return p_cell_population;
}
