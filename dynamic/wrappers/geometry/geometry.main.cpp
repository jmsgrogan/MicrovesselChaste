#include <pybind11/pybind11.h>
#include "Vertex2.cppwg.hpp"
#include "Vertex3.cppwg.hpp"
#include "Polygon2.cppwg.hpp"
#include "Polygon3.cppwg.hpp"
#include "Facet2.cppwg.hpp"
#include "Facet3.cppwg.hpp"
#include "Part2.cppwg.hpp"
#include "Part3.cppwg.hpp"
#include "MappableGridGenerator2.cppwg.hpp"
#include "MappableGridGenerator3.cppwg.hpp"
#include "NetworkToSurface2.cppwg.hpp"
#include "NetworkToSurface3.cppwg.hpp"
#include "VesselSurfaceGenerator2.cppwg.hpp"
#include "VesselSurfaceGenerator3.cppwg.hpp"
#include "BoundaryExtractor.cppwg.hpp"
#include "SurfaceCleaner.cppwg.hpp"
#include "GeometryFormat.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_geometry, m)
{
    register_Vertex2_class(m);
    register_Vertex3_class(m);
    register_Polygon2_class(m);
    register_Polygon3_class(m);
    register_Facet2_class(m);
    register_Facet3_class(m);
    register_Part2_class(m);
    register_Part3_class(m);
    register_MappableGridGenerator2_class(m);
    register_MappableGridGenerator3_class(m);
    register_NetworkToSurface2_class(m);
    register_NetworkToSurface3_class(m);
    register_VesselSurfaceGenerator2_class(m);
    register_VesselSurfaceGenerator3_class(m);
    register_BoundaryExtractor_class(m);
    register_SurfaceCleaner_class(m);
    register_GeometryFormat_class(m);
}
