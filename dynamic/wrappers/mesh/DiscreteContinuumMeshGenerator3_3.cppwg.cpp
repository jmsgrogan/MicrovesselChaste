#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"

#include "DiscreteContinuumMeshGenerator3_3.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumMeshGenerator<3,3 > DiscreteContinuumMeshGenerator3_3;
;

void register_DiscreteContinuumMeshGenerator3_3_class(py::module &m){
py::class_<DiscreteContinuumMeshGenerator3_3    >(m, "DiscreteContinuumMeshGenerator3_3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumMeshGenerator<3, 3> >(*)()) &DiscreteContinuumMeshGenerator3_3::Create, 
            " "  )
        .def(
            "GetMesh", 
            (::std::shared_ptr<DiscreteContinuumMesh<3, 3> >(DiscreteContinuumMeshGenerator3_3::*)()) &DiscreteContinuumMeshGenerator3_3::GetMesh, 
            " "  )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::std::shared_ptr<Part<3> >)) &DiscreteContinuumMeshGenerator3_3::SetDomain, 
            " " , py::arg("pDomain") )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::vtkSmartPointer<vtkPolyData>)) &DiscreteContinuumMeshGenerator3_3::SetDomain, 
            " " , py::arg("pDomain") )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::std::string const &)) &DiscreteContinuumMeshGenerator3_3::SetDomain, 
            " " , py::arg("rPathToStl") )
        .def(
            "SetMaxElementArea", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::QVolume)) &DiscreteContinuumMeshGenerator3_3::SetMaxElementArea, 
            " " , py::arg("maxElementArea") )
        .def(
            "SetHoles", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::std::vector<DimensionalChastePoint<3>, std::allocator<DimensionalChastePoint<3> > >)) &DiscreteContinuumMeshGenerator3_3::SetHoles, 
            " " , py::arg("holes") )
        .def(
            "SetRegionMarkers", 
            (void(DiscreteContinuumMeshGenerator3_3::*)(::std::vector<DimensionalChastePoint<3>, std::allocator<DimensionalChastePoint<3> > >)) &DiscreteContinuumMeshGenerator3_3::SetRegionMarkers, 
            " " , py::arg("regionMarkers") )
        .def(
            "Update", 
            (void(DiscreteContinuumMeshGenerator3_3::*)()) &DiscreteContinuumMeshGenerator3_3::Update, 
            " "  )
    ;
}
