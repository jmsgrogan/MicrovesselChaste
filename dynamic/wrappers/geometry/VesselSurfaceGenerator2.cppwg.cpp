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

#include "PythonObjectConverters.hpp"
#include "VesselSurfaceGenerator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VesselSurfaceGenerator<2 > VesselSurfaceGenerator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselSurfaceGenerator2_class(py::module &m){
py::class_<VesselSurfaceGenerator2  , std::shared_ptr<VesselSurfaceGenerator2 >   >(m, "VesselSurfaceGenerator2")
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
            (::std::vector<Vertex<2>, std::allocator<Vertex<2> > >(VesselSurfaceGenerator2::*)()) &VesselSurfaceGenerator2::GetHoles, 
            " "  )
        .def(
            "GetVtkSurface", 
            (::vtkSmartPointer<vtkPolyData>(VesselSurfaceGenerator2::*)()) &VesselSurfaceGenerator2::GetVtkSurface, 
            " "  )
    ;
}
