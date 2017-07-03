#include <pybind11/pybind11.h>
#include "MicrovesselSolver2.cppwg.hpp"
#include "MicrovesselSolver3.cppwg.hpp"
#include "MicrovesselSimulationModifier2.cppwg.hpp"
#include "MicrovesselSimulationModifier3.cppwg.hpp"
#include "AbstractMicrovesselModifier2.cppwg.hpp"
#include "AbstractMicrovesselModifier3.cppwg.hpp"
#include "VtkSceneMicrovesselModifier2.cppwg.hpp"
#include "VtkSceneMicrovesselModifier3.cppwg.hpp"
#include "Owen2011TrackingModifier2.cppwg.hpp"
#include "Owen2011TrackingModifier3.cppwg.hpp"
#include "AbstractCellBasedSimulationModifier2_2.cppwg.hpp"
#include "AbstractCellBasedSimulationModifier3_3.cppwg.hpp"
#include "CornealMicropocketSimulation2.cppwg.hpp"
#include "CornealMicropocketSimulation3.cppwg.hpp"
#include "DomainType.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_simulation, m)
{
    register_MicrovesselSolver2_class(m);
    register_MicrovesselSolver3_class(m);
    register_MicrovesselSimulationModifier2_class(m);
    register_MicrovesselSimulationModifier3_class(m);
    register_AbstractMicrovesselModifier2_class(m);
    register_AbstractMicrovesselModifier3_class(m);
    register_VtkSceneMicrovesselModifier2_class(m);
    register_VtkSceneMicrovesselModifier3_class(m);
    register_Owen2011TrackingModifier2_class(m);
    register_Owen2011TrackingModifier3_class(m);
    register_AbstractCellBasedSimulationModifier2_2_class(m);
    register_AbstractCellBasedSimulationModifier3_3_class(m);
    register_CornealMicropocketSimulation2_class(m);
    register_CornealMicropocketSimulation3_class(m);
    register_DomainType_class(m);
}
