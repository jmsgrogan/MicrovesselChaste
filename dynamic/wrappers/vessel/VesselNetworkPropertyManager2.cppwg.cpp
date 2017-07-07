#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNetworkPropertyManager.hpp"

#include "VesselNetworkPropertyManager2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkPropertyManager<2 > VesselNetworkPropertyManager2;
;

void register_VesselNetworkPropertyManager2_class(py::module &m){
py::class_<VesselNetworkPropertyManager2    >(m, "VesselNetworkPropertyManager2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkPropertyManager<2> >(*)()) &VesselNetworkPropertyManager2::Create, 
            " "  )
        .def_static(
            "AssignInflows", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, ::DimensionalChastePoint<2>, ::QLength)) &VesselNetworkPropertyManager2::AssignInflows, 
            " " , py::arg("pNetwork"), py::arg("location"), py::arg("searchRadius") )
        .def_static(
            "AssignOutflows", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, ::DimensionalChastePoint<2>, ::QLength)) &VesselNetworkPropertyManager2::AssignOutflows, 
            " " , py::arg("pNetwork"), py::arg("location"), py::arg("searchRadius") )
        .def_static(
            "CopySegmentFlowProperties", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, unsigned int)) &VesselNetworkPropertyManager2::CopySegmentFlowProperties, 
            " " , py::arg("pNetwork"), py::arg("index") = 0 )
        .def_static(
            "GetInflowNodes", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkPropertyManager2::GetInflowNodes, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetOutflowNodes", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkPropertyManager2::GetOutflowNodes, 
            " " , py::arg("pNetwork") )
        .def_static(
            "SetNodeRadii", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, ::QLength)) &VesselNetworkPropertyManager2::SetNodeRadii, 
            " " , py::arg("pNetwork"), py::arg("radius") )
        .def_static(
            "SetNodeRadiiFromSegments", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkPropertyManager2::SetNodeRadiiFromSegments, 
            " " , py::arg("pNetwork") )
        .def_static(
            "SetInflowPressures", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, ::QPressure)) &VesselNetworkPropertyManager2::SetInflowPressures, 
            " " , py::arg("pNetwork"), py::arg("pressure") )
        .def_static(
            "SetOutflowPressures", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, ::QPressure)) &VesselNetworkPropertyManager2::SetOutflowPressures, 
            " " , py::arg("pNetwork"), py::arg("pressure") )
        .def_static(
            "SetSegmentProperties", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, ::std::shared_ptr<VesselSegment<2> >)) &VesselNetworkPropertyManager2::SetSegmentProperties, 
            " " , py::arg("pNetwork"), py::arg("prototype") )
        .def_static(
            "SetSegmentRadii", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, ::QLength)) &VesselNetworkPropertyManager2::SetSegmentRadii, 
            " " , py::arg("pNetwork"), py::arg("radius") )
        .def_static(
            "SetSegmentViscosity", 
            (void(*)(::std::shared_ptr<VesselNetwork<2> >, ::QDynamicViscosity)) &VesselNetworkPropertyManager2::SetSegmentViscosity, 
            " " , py::arg("pNetwork"), py::arg("viscosity") )
    ;
}
