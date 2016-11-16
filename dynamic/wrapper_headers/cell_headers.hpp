<<<<<<< HEAD
#include "CancerCellMutationState.hpp"
#include "Owen2011OxygenBasedCellCycleModel.hpp"


=======
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

template class Owen11CellPopulationGenerator<2>;
template class Owen11CellPopulationGenerator<3>;
template class CaBasedCellPopulation<2>;
template class CaBasedCellPopulation<3>;
>>>>>>> 771a962055d447a8738a2e7efbc60beb1eaaa477
