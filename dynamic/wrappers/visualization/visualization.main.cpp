#include <pybind11/pybind11.h>
#include "MicrovesselVtkScene2.cppwg.hpp"
#include "MicrovesselVtkScene3.cppwg.hpp"
#include "AbstractActorGenerator2.cppwg.hpp"
#include "AbstractActorGenerator3.cppwg.hpp"
#include "CellPopulationActorGenerator2.cppwg.hpp"
#include "CellPopulationActorGenerator3.cppwg.hpp"
#include "DiscreteContinuumMeshActorGenerator2.cppwg.hpp"
#include "DiscreteContinuumMeshActorGenerator3.cppwg.hpp"
#include "PartActorGenerator2.cppwg.hpp"
#include "PartActorGenerator3.cppwg.hpp"
#include "RegularGridActorGenerator2.cppwg.hpp"
#include "RegularGridActorGenerator3.cppwg.hpp"
#include "VesselNetworkActorGenerator2.cppwg.hpp"
#include "VesselNetworkActorGenerator3.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_visualization, m)
{
    register_MicrovesselVtkScene2_class(m);
    register_MicrovesselVtkScene3_class(m);
    register_AbstractActorGenerator2_class(m);
    register_AbstractActorGenerator3_class(m);
    register_CellPopulationActorGenerator2_class(m);
    register_CellPopulationActorGenerator3_class(m);
    register_DiscreteContinuumMeshActorGenerator2_class(m);
    register_DiscreteContinuumMeshActorGenerator3_class(m);
    register_PartActorGenerator2_class(m);
    register_PartActorGenerator3_class(m);
    register_RegularGridActorGenerator2_class(m);
    register_RegularGridActorGenerator3_class(m);
    register_VesselNetworkActorGenerator2_class(m);
    register_VesselNetworkActorGenerator3_class(m);
}
