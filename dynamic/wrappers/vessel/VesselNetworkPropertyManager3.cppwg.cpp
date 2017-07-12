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
#include "VesselNetworkPropertyManager.hpp"

#include "VesselNetworkPropertyManager3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkPropertyManager<3 > VesselNetworkPropertyManager3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkPropertyManager3_class(py::module &m){
py::class_<VesselNetworkPropertyManager3  , std::shared_ptr<VesselNetworkPropertyManager3 >   >(m, "VesselNetworkPropertyManager3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkPropertyManager<3> >(*)()) &VesselNetworkPropertyManager3::Create, 
            " "  )
        .def_static(
            "AssignInflows", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, ::Vertex<3>, ::QLength)) &VesselNetworkPropertyManager3::AssignInflows, 
            " " , py::arg("pNetwork"), py::arg("location"), py::arg("searchRadius") )
        .def_static(
            "AssignOutflows", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, ::Vertex<3>, ::QLength)) &VesselNetworkPropertyManager3::AssignOutflows, 
            " " , py::arg("pNetwork"), py::arg("location"), py::arg("searchRadius") )
        .def_static(
            "CopySegmentFlowProperties", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, unsigned int)) &VesselNetworkPropertyManager3::CopySegmentFlowProperties, 
            " " , py::arg("pNetwork"), py::arg("index") = 0 )
        .def_static(
            "GetInflowNodes", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkPropertyManager3::GetInflowNodes, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetOutflowNodes", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkPropertyManager3::GetOutflowNodes, 
            " " , py::arg("pNetwork") )
        .def_static(
            "SetNodeRadii", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, ::QLength)) &VesselNetworkPropertyManager3::SetNodeRadii, 
            " " , py::arg("pNetwork"), py::arg("radius") )
        .def_static(
            "SetNodeRadiiFromSegments", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkPropertyManager3::SetNodeRadiiFromSegments, 
            " " , py::arg("pNetwork") )
        .def_static(
            "SetInflowPressures", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, ::QPressure)) &VesselNetworkPropertyManager3::SetInflowPressures, 
            " " , py::arg("pNetwork"), py::arg("pressure") )
        .def_static(
            "SetOutflowPressures", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, ::QPressure)) &VesselNetworkPropertyManager3::SetOutflowPressures, 
            " " , py::arg("pNetwork"), py::arg("pressure") )
        .def_static(
            "SetSegmentProperties", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, ::std::shared_ptr<VesselSegment<3> >)) &VesselNetworkPropertyManager3::SetSegmentProperties, 
            " " , py::arg("pNetwork"), py::arg("prototype") )
        .def_static(
            "SetSegmentRadii", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, ::QLength)) &VesselNetworkPropertyManager3::SetSegmentRadii, 
            " " , py::arg("pNetwork"), py::arg("radius") )
        .def_static(
            "SetSegmentViscosity", 
            (void(*)(::std::shared_ptr<VesselNetwork<3> >, ::QDynamicViscosity)) &VesselNetworkPropertyManager3::SetSegmentViscosity, 
            " " , py::arg("pNetwork"), py::arg("viscosity") )
    ;
}
