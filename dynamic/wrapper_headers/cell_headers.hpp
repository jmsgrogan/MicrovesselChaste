#include "AbstractCellKiller.hpp"
#include "AbstractCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "QuiescentCancerCellMutationState.hpp"
#include "StalkCellMutationState.hpp"
#include "TipCellMutationState.hpp"
#include "VesselCellMutationState.hpp"
#include "MacrophageMutationState.hpp"
#include "Owen2011OxygenBasedCellCycleModel.hpp"
#include "Owen11CellPopulationGenerator.hpp"
#include "CaBasedCellPopulation.hpp"
#include "LQRadiotherapyCellKiller.hpp"

template class Owen11CellPopulationGenerator<2>;
template class Owen11CellPopulationGenerator<3>;
template class CaBasedCellPopulation<2>;
template class CaBasedCellPopulation<3>;
template class LQRadiotherapyCellKiller<2>;
template class LQRadiotherapyCellKiller<3>;
template class AbstractCellKiller<2>;
template class AbstractCellKiller<3>;
