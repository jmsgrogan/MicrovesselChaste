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

#include "DiscreteContinuumMeshGenerator2_2.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumMeshGenerator<2,2 > DiscreteContinuumMeshGenerator2_2;
;

void register_DiscreteContinuumMeshGenerator2_2_class(py::module &m){
py::class_<DiscreteContinuumMeshGenerator2_2    >(m, "DiscreteContinuumMeshGenerator2_2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumMeshGenerator<2, 2> >(*)()) &DiscreteContinuumMeshGenerator2_2::Create, 
            " "  )
        .def(
            "GetMesh", 
            (::std::shared_ptr<DiscreteContinuumMesh<2, 2> >(DiscreteContinuumMeshGenerator2_2::*)()) &DiscreteContinuumMeshGenerator2_2::GetMesh, 
            " "  )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::std::shared_ptr<Part<2> >)) &DiscreteContinuumMeshGenerator2_2::SetDomain, 
            " " , py::arg("pDomain") )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::vtkSmartPointer<vtkPolyData>)) &DiscreteContinuumMeshGenerator2_2::SetDomain, 
            " " , py::arg("pDomain") )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::std::string const &)) &DiscreteContinuumMeshGenerator2_2::SetDomain, 
            " " , py::arg("rPathToStl") )
        .def(
            "SetMaxElementArea", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::QVolume)) &DiscreteContinuumMeshGenerator2_2::SetMaxElementArea, 
            " " , py::arg("maxElementArea") )
        .def(
            "SetHoles", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >)) &DiscreteContinuumMeshGenerator2_2::SetHoles, 
            " " , py::arg("holes") )
        .def(
            "SetRegionMarkers", 
            (void(DiscreteContinuumMeshGenerator2_2::*)(::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >)) &DiscreteContinuumMeshGenerator2_2::SetRegionMarkers, 
            " " , py::arg("regionMarkers") )
        .def(
            "Update", 
            (void(DiscreteContinuumMeshGenerator2_2::*)()) &DiscreteContinuumMeshGenerator2_2::Update, 
            " "  )
    ;
}
