#include <pybind11/pybind11.h>
#include "FlowSolver2.cppwg.hpp"
#include "FlowSolver3.cppwg.hpp"
#include "WallShearStressCalculator2.cppwg.hpp"
#include "WallShearStressCalculator3.cppwg.hpp"
#include "VesselImpedanceCalculator2.cppwg.hpp"
#include "VesselImpedanceCalculator3.cppwg.hpp"
#include "BetteridgeHaematocritSolver2.cppwg.hpp"
#include "BetteridgeHaematocritSolver3.cppwg.hpp"
#include "AbstractVesselNetworkCalculator2.cppwg.hpp"
#include "AbstractVesselNetworkCalculator3.cppwg.hpp"
#include "MechanicalStimulusCalculator2.cppwg.hpp"
#include "MechanicalStimulusCalculator3.cppwg.hpp"
#include "MetabolicStimulusCalculator2.cppwg.hpp"
#include "MetabolicStimulusCalculator3.cppwg.hpp"
#include "RadiusCalculator2.cppwg.hpp"
#include "RadiusCalculator3.cppwg.hpp"
#include "ShrinkingStimulusCalculator2.cppwg.hpp"
#include "ShrinkingStimulusCalculator3.cppwg.hpp"
#include "ViscosityCalculator2.cppwg.hpp"
#include "ViscosityCalculator3.cppwg.hpp"
#include "AbstractStructuralAdaptationSolver2.cppwg.hpp"
#include "AbstractStructuralAdaptationSolver3.cppwg.hpp"
#include "StructuralAdaptationSolver2.cppwg.hpp"
#include "StructuralAdaptationSolver3.cppwg.hpp"
#include "AbstractHaematocritSolver2.cppwg.hpp"
#include "AbstractHaematocritSolver3.cppwg.hpp"
#include "AlarconHaematocritSolver2.cppwg.hpp"
#include "AlarconHaematocritSolver3.cppwg.hpp"
#include "ConstantHaematocritSolver2.cppwg.hpp"
#include "ConstantHaematocritSolver3.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_flow, m)
{
    register_FlowSolver2_class(m);
    register_FlowSolver3_class(m);
    register_WallShearStressCalculator2_class(m);
    register_WallShearStressCalculator3_class(m);
    register_VesselImpedanceCalculator2_class(m);
    register_VesselImpedanceCalculator3_class(m);
    register_BetteridgeHaematocritSolver2_class(m);
    register_BetteridgeHaematocritSolver3_class(m);
    register_AbstractVesselNetworkCalculator2_class(m);
    register_AbstractVesselNetworkCalculator3_class(m);
    register_MechanicalStimulusCalculator2_class(m);
    register_MechanicalStimulusCalculator3_class(m);
    register_MetabolicStimulusCalculator2_class(m);
    register_MetabolicStimulusCalculator3_class(m);
    register_RadiusCalculator2_class(m);
    register_RadiusCalculator3_class(m);
    register_ShrinkingStimulusCalculator2_class(m);
    register_ShrinkingStimulusCalculator3_class(m);
    register_ViscosityCalculator2_class(m);
    register_ViscosityCalculator3_class(m);
    register_AbstractStructuralAdaptationSolver2_class(m);
    register_AbstractStructuralAdaptationSolver3_class(m);
    register_StructuralAdaptationSolver2_class(m);
    register_StructuralAdaptationSolver3_class(m);
    register_AbstractHaematocritSolver2_class(m);
    register_AbstractHaematocritSolver3_class(m);
    register_AlarconHaematocritSolver2_class(m);
    register_AlarconHaematocritSolver3_class(m);
    register_ConstantHaematocritSolver2_class(m);
    register_ConstantHaematocritSolver3_class(m);
}
