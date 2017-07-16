#ifndef chaste_project_MicrovesselChaste_HEADERS_HPP_
#define chaste_project_MicrovesselChaste_HEADERS_HPP_

// Includes
#include "MicrovesselVtkScene.hpp"
#include "AbstractActorGenerator.hpp"
#include "CellPopulationActorGenerator.hpp"
#include "DiscreteContinuumMeshActorGenerator.hpp"
#include "PartActorGenerator.hpp"
#include "RegularGridActorGenerator.hpp"
#include "VesselNetworkActorGenerator.hpp"

// Instantiate Template Classes 
template class MicrovesselVtkScene<2 >;
template class MicrovesselVtkScene<3 >;
template class AbstractActorGenerator<2 >;
template class AbstractActorGenerator<3 >;
template class CellPopulationActorGenerator<2 >;
template class CellPopulationActorGenerator<3 >;
template class DiscreteContinuumMeshActorGenerator<2 >;
template class DiscreteContinuumMeshActorGenerator<3 >;
template class PartActorGenerator<2 >;
template class PartActorGenerator<3 >;
template class RegularGridActorGenerator<2 >;
template class RegularGridActorGenerator<3 >;
template class VesselNetworkActorGenerator<2 >;
template class VesselNetworkActorGenerator<3 >;

// Typedef for nicer naming
namespace cppwg{ 
typedef MicrovesselVtkScene<2 > MicrovesselVtkScene2;
typedef MicrovesselVtkScene<3 > MicrovesselVtkScene3;
typedef AbstractActorGenerator<2 > AbstractActorGenerator2;
typedef AbstractActorGenerator<3 > AbstractActorGenerator3;
typedef CellPopulationActorGenerator<2 > CellPopulationActorGenerator2;
typedef CellPopulationActorGenerator<3 > CellPopulationActorGenerator3;
typedef DiscreteContinuumMeshActorGenerator<2 > DiscreteContinuumMeshActorGenerator2;
typedef DiscreteContinuumMeshActorGenerator<3 > DiscreteContinuumMeshActorGenerator3;
typedef PartActorGenerator<2 > PartActorGenerator2;
typedef PartActorGenerator<3 > PartActorGenerator3;
typedef RegularGridActorGenerator<2 > RegularGridActorGenerator2;
typedef RegularGridActorGenerator<3 > RegularGridActorGenerator3;
typedef VesselNetworkActorGenerator<2 > VesselNetworkActorGenerator2;
typedef VesselNetworkActorGenerator<3 > VesselNetworkActorGenerator3;
}

#endif // chaste_project_MicrovesselChaste_HEADERS_HPP_
