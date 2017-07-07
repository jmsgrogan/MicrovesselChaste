#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselSurfaceGenerator.hpp"

#include "VesselSurfaceGenerator2.cppwg.hpp"

namespace py = pybind11;
typedef VesselSurfaceGenerator<2 > VesselSurfaceGenerator2;
;

void register_VesselSurfaceGenerator2_class(py::module &m){
py::class_<VesselSurfaceGenerator2    >(m, "VesselSurfaceGenerator2")
        .def(py::init<::std::shared_ptr<VesselNetwork<2> > >(), py::arg("pVesselNetwork"))
        .def(
            "GetSurface", 
            (::std::vector<std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > >, std::allocator<std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > > > >(VesselSurfaceGenerator2::*)()) &VesselSurfaceGenerator2::GetSurface, 
            " "  )
        .def(
            "GetSurfacePolygons", 
            (::std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > >(VesselSurfaceGenerator2::*)()) &VesselSurfaceGenerator2::GetSurfacePolygons, 
            " "  )
        .def(
            "GetHoles", 
            (::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >(VesselSurfaceGenerator2::*)()) &VesselSurfaceGenerator2::GetHoles, 
            " "  )
        .def(
            "GetVtkSurface", 
            (::vtkSmartPointer<vtkPolyData>(VesselSurfaceGenerator2::*)()) &VesselSurfaceGenerator2::GetVtkSurface, 
            " "  )
    ;
}
