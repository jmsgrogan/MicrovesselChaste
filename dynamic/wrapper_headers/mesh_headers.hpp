#include "RegularGrid.hpp"
#include "RegularGridWriter.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "MeshReader.hpp"
#include "MultiFormatMeshWriter.hpp"
#include "DimensionalChastePoint.hpp"

inline int Instantiation()
{
    return
            sizeof(RegularGrid<3>) +
            sizeof(RegularGrid<2>) +
            sizeof(DiscreteContinuumMesh<3, 3>) +
            sizeof(DiscreteContinuumMesh<2, 2>) +
            sizeof(DiscreteContinuumMeshGenerator<3,3>)+
            sizeof(DiscreteContinuumMeshGenerator<2,2>)+
            sizeof(MultiFormatMeshWriter<3>)+
            sizeof(MultiFormatMeshWriter<2>)+
            sizeof(DimensionalChastePoint<3>)+
            sizeof(DimensionalChastePoint<2>);
}
