#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "vtkPolyData.h"
#include "VesselSurfaceGenerator.hpp"

#include "VesselSurfaceGenerator3.cppwg.hpp"

namespace py = pybind11;
typedef VesselSurfaceGenerator<3 > VesselSurfaceGenerator3;
;

void register_VesselSurfaceGenerator3_class(py::module &m){
py::class_<VesselSurfaceGenerator3    >(m, "VesselSurfaceGenerator3")
        .def(py::init<::std::shared_ptr<VesselNetwork<3> > >(), py::arg("pVesselNetwork"))
        .def(
            "GetSurface", 
            (::std::vector<std::vector<std::shared_ptr<Polygon<3> >, std::allocator<std::shared_ptr<Polygon<3> > > >, std::allocator<std::vector<std::shared_ptr<Polygon<3> >, std::allocator<std::shared_ptr<Polygon<3> > > > > >(VesselSurfaceGenerator3::*)()) &VesselSurfaceGenerator3::GetSurface, 
            " "  )
        .def(
            "GetSurfacePolygons", 
            (::std::vector<std::shared_ptr<Polygon<3> >, std::allocator<std::shared_ptr<Polygon<3> > > >(VesselSurfaceGenerator3::*)()) &VesselSurfaceGenerator3::GetSurfacePolygons, 
            " "  )
        .def(
            "GetHoles", 
            (::std::vector<Vertex<3>, std::allocator<Vertex<3> > >(VesselSurfaceGenerator3::*)()) &VesselSurfaceGenerator3::GetHoles, 
            " "  )
        .def(
            "GetVtkSurface", 
            (::vtkSmartPointer<vtkPolyData>(VesselSurfaceGenerator3::*)()) &VesselSurfaceGenerator3::GetVtkSurface, 
            " "  )
    ;
}
