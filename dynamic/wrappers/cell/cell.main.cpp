#include <pybind11/pybind11.h>
#include "CancerCellMutationState.cppwg.hpp"
#include "QuiescentCancerCellMutationState.cppwg.hpp"
#include "StalkCellMutationState.cppwg.hpp"
#include "TipCellMutationState.cppwg.hpp"
#include "VesselCellMutationState.cppwg.hpp"
#include "MacrophageMutationState.cppwg.hpp"
#include "Owen2011OxygenBasedCellCycleModel.cppwg.hpp"
#include "Owen11CellPopulationGenerator2.cppwg.hpp"
#include "Owen11CellPopulationGenerator3.cppwg.hpp"
#include "CaBasedCellPopulation2.cppwg.hpp"
#include "CaBasedCellPopulation3.cppwg.hpp"
#include "LQRadiotherapyCellKiller2.cppwg.hpp"
#include "LQRadiotherapyCellKiller3.cppwg.hpp"
#include "Owen11CaBasedDivisionRule2.cppwg.hpp"
#include "Owen11CaBasedDivisionRule3.cppwg.hpp"
#include "Owen11CaUpdateRule2.cppwg.hpp"
#include "Owen11CaUpdateRule3.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_cell, m)
{
    register_CancerCellMutationState_class(m);
    register_QuiescentCancerCellMutationState_class(m);
    register_StalkCellMutationState_class(m);
    register_TipCellMutationState_class(m);
    register_VesselCellMutationState_class(m);
    register_MacrophageMutationState_class(m);
    register_Owen2011OxygenBasedCellCycleModel_class(m);
    register_Owen11CellPopulationGenerator2_class(m);
    register_Owen11CellPopulationGenerator3_class(m);
    register_CaBasedCellPopulation2_class(m);
    register_CaBasedCellPopulation3_class(m);
    register_LQRadiotherapyCellKiller2_class(m);
    register_LQRadiotherapyCellKiller3_class(m);
    register_Owen11CaBasedDivisionRule2_class(m);
    register_Owen11CaBasedDivisionRule3_class(m);
    register_Owen11CaUpdateRule2_class(m);
    register_Owen11CaUpdateRule3_class(m);
}
