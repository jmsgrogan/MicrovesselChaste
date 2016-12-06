#include "MicrovesselVtkScene.hpp"
#include "CellPopulationActorGenerator.hpp"
#include "DiscreteContinuumMeshActorGenerator.hpp"
#include "PartActorGenerator.hpp"
#include "RegularGridActorGenerator.hpp"
#include "VesselNetworkActorGenerator.hpp"
#include "AbstractActorGenerator.hpp"
#include "AbstractMicrovesselModifier.hpp"
#include "VtkSceneMicrovesselModifier.hpp"

template class MicrovesselVtkScene<2>;
template class CellPopulationActorGenerator<2>;
template class DiscreteContinuumMeshActorGenerator<2>;
template class PartActorGenerator<2>;
template class RegularGridActorGenerator<2>;
template class VesselNetworkActorGenerator<2>;
template class AbstractActorGenerator<2>;
template class AbstractMicrovesselModifier<2>;
template class VtkSceneMicrovesselModifier<2>;
template class MicrovesselVtkScene<3>;
template class CellPopulationActorGenerator<3>;
template class DiscreteContinuumMeshActorGenerator<3>;
template class PartActorGenerator<3>;
template class RegularGridActorGenerator<3>;
template class VesselNetworkActorGenerator<3>;
template class AbstractActorGenerator<3>;
template class AbstractMicrovesselModifier<3>;
template class VtkSceneMicrovesselModifier<3>;
