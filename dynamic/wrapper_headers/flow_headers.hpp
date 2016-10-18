#include "FlowSolver.hpp"
#include "WallShearStressCalculator.hpp"
#include "VesselImpedanceCalculator.hpp"
#include "BetteridgeHaematocritSolver.hpp"
#include "AbstractVesselNetworkCalculator.hpp"
#include "MechanicalStimulusCalculator.hpp"
#include "MetabolicStimulusCalculator.hpp"
#include "RadiusCalculator.hpp"
#include "ShrinkingStimulusCalculator.hpp"
#include "ViscosityCalculator.hpp"
#include "WallShearStressCalculator.hpp"
#include "AbstractStructuralAdaptationSolver.hpp"
#include "StructuralAdaptationSolver.hpp"
#include "AbstractHaematocritSolver.hpp"
#include "AlarconHaematocritSolver.hpp"
#include "ConstantHaematocritSolver.hpp"

template class FlowSolver<2>;
template class FlowSolver<3>;
template class WallShearStressCalculator<2>;
template class WallShearStressCalculator<3>;
template class VesselImpedanceCalculator<3>;
template class BetteridgeHaematocritSolver<3>;
template class AbstractVesselNetworkCalculator<3>;
template class MechanicalStimulusCalculator<3>;
template class MetabolicStimulusCalculator<3>;
template class RadiusCalculator<3>;
template class ShrinkingStimulusCalculator<3>;
template class ViscosityCalculator<3>;
template class WallShearStressCalculator<3>;
template class AbstractStructuralAdaptationSolver<3>;
template class StructuralAdaptationSolver<3>;
template class AbstractHaematocritSolver<3>;
template class AlarconHaematocritSolver<3>;
template class ConstantHaematocritSolver<3>;
template class VesselImpedanceCalculator<2>;
template class BetteridgeHaematocritSolver<2>;
template class AbstractVesselNetworkCalculator<2>;
template class MechanicalStimulusCalculator<2>;
template class MetabolicStimulusCalculator<2>;
template class RadiusCalculator<2>;
template class ShrinkingStimulusCalculator<2>;
template class ViscosityCalculator<2>;
template class WallShearStressCalculator<2>;
template class AbstractStructuralAdaptationSolver<2>;
template class StructuralAdaptationSolver<2>;
template class AbstractHaematocritSolver<2>;
template class AlarconHaematocritSolver<2>;
template class ConstantHaematocritSolver<2>;

