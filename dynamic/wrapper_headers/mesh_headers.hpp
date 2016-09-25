#include "RegularGrid.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DimensionalChastePoint.hpp"

inline int Instantiation()
{
    return
            sizeof(RegularGrid<3, 3>) +
            sizeof(DiscreteContinuumMesh<3, 3>) +
            sizeof(DimensionalChastePoint<3>)+
            sizeof(DimensionalChastePoint<2>);
}
