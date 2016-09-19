#include "FlowSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "VesselImpedanceCalculator.hpp"
#include "BetteridgeHaematocritSolver.hpp"

template class FlowSolver<2>;
template class FlowSolver<3>;
template class WallShearStressCalculator<2>;
template class WallShearStressCalculator<3>;
template class VesselImpedanceCalculator<3>;
template class BetteridgeHaematocritSolver<3>;
