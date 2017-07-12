#include <pybind11/pybind11.h>
#include "AbstractMesh2_2.cppwg.hpp"
#include "AbstractMesh3_3.cppwg.hpp"
#include "AbstractTetrahedralMesh2_2.cppwg.hpp"
#include "AbstractTetrahedralMesh3_3.cppwg.hpp"
#include "AbstractOdeSystem.cppwg.hpp"
#include "TetrahedralMesh2_2.cppwg.hpp"
#include "TetrahedralMesh3_3.cppwg.hpp"
#include "AbstractDiscreteContinuumGrid2_2.cppwg.hpp"
#include "AbstractDiscreteContinuumGrid3_3.cppwg.hpp"
#include "DiscreteContinuumMesh2_2.cppwg.hpp"
#include "DiscreteContinuumMesh3_3.cppwg.hpp"
#include "DiscreteContinuumMeshGenerator2_2.cppwg.hpp"
#include "DiscreteContinuumMeshGenerator3_3.cppwg.hpp"
#include "GridCalculator2.cppwg.hpp"
#include "GridCalculator3.cppwg.hpp"
#include "MeshReader.cppwg.hpp"
#include "MultiFormatMeshWriter2.cppwg.hpp"
#include "MultiFormatMeshWriter3.cppwg.hpp"
#include "RegularGrid2.cppwg.hpp"
#include "RegularGrid3.cppwg.hpp"
#include "RegularGridWriter.cppwg.hpp"
#include "MeshFormat.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_mesh, m)
{
    register_AbstractMesh2_2_class(m);
    register_AbstractMesh3_3_class(m);
    register_AbstractTetrahedralMesh2_2_class(m);
    register_AbstractTetrahedralMesh3_3_class(m);
    register_AbstractOdeSystem_class(m);
    register_TetrahedralMesh2_2_class(m);
    register_TetrahedralMesh3_3_class(m);
    register_AbstractDiscreteContinuumGrid2_2_class(m);
    register_AbstractDiscreteContinuumGrid3_3_class(m);
    register_DiscreteContinuumMesh2_2_class(m);
    register_DiscreteContinuumMesh3_3_class(m);
    register_DiscreteContinuumMeshGenerator2_2_class(m);
    register_DiscreteContinuumMeshGenerator3_3_class(m);
    register_GridCalculator2_class(m);
    register_GridCalculator3_class(m);
    register_MeshReader_class(m);
    register_MultiFormatMeshWriter2_class(m);
    register_MultiFormatMeshWriter3_class(m);
    register_RegularGrid2_class(m);
    register_RegularGrid3_class(m);
    register_RegularGridWriter_class(m);
    register_MeshFormat_class(m);
}
