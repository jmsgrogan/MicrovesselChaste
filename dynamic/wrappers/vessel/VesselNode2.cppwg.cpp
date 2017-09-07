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
#include "VesselNode.hpp"

#include "PythonObjectConverters.hpp"
#include "VesselNode2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef VesselNode<2 > VesselNode2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_;

class VesselNode2_Overloads : public VesselNode2{
    public:
    using VesselNode2::VesselNode;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD(
            _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_,
            VesselNode2,
            GetOutputData,
            );
    }

};
void register_VesselNode2_class(py::module &m){
py::class_<VesselNode2 , VesselNode2_Overloads , std::shared_ptr<VesselNode2 >  , AbstractVesselNetworkComponent<2>  >(m, "VesselNode2")
        .def(py::init<::QLength, ::QLength, ::QLength >(), py::arg("v1") = 0_m, py::arg("v2") = 0_m, py::arg("v3") = 0_m)
        .def(py::init<::Vertex<2> const & >(), py::arg("rLocation"))
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<2> >(*)(::QLength, ::QLength, ::QLength)) &VesselNode2::Create, 
            " " , py::arg("v1"), py::arg("v2") = 0_m, py::arg("v3") = 0_m )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<2> >(*)(::Vertex<2> const &)) &VesselNode2::Create, 
            " " , py::arg("location") )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<2> >(*)(::VesselNode<2> const &)) &VesselNode2::Create, 
            " " , py::arg("rExistingNode") )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<2> >(*)(::std::shared_ptr<VesselNode<2> >)) &VesselNode2::Create, 
            " " , py::arg("pExistingNode") )
        .def(
            "GetComparisonId", 
            (unsigned int(VesselNode2::*)()) &VesselNode2::GetComparisonId, 
            " "  )
        .def(
            "GetDistance", 
            (::QLength(VesselNode2::*)(::Vertex<2> const &) const ) &VesselNode2::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetFlowProperties", 
            (::std::shared_ptr<NodeFlowProperties<2> >(VesselNode2::*)() const ) &VesselNode2::GetFlowProperties, 
            " "  )
        .def(
            "GetChemicalProperties", 
            (::std::shared_ptr<NodeChemicalProperties<2> >(VesselNode2::*)() const ) &VesselNode2::GetChemicalProperties, 
            " "  )
        .def(
            "rGetLocation", 
            (::Vertex<2> const &(VesselNode2::*)() const ) &VesselNode2::rGetLocation, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "GetNumberOfSegments", 
            (unsigned int(VesselNode2::*)() const ) &VesselNode2::GetNumberOfSegments, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(VesselNode2::*)()) &VesselNode2::GetOutputData, 
            " "  )
        .def(
            "GetSegment", 
            (::std::shared_ptr<VesselSegment<2> >(VesselNode2::*)(unsigned int) const ) &VesselNode2::GetSegment, 
            " " , py::arg("index") )
        .def(
            "GetSegments", 
            (::std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >(VesselNode2::*)() const ) &VesselNode2::GetSegments, 
            " "  )
        .def(
            "GetGlobalIndex", 
            (unsigned int(VesselNode2::*)()) &VesselNode2::GetGlobalIndex, 
            " "  )
        .def(
            "GetLocalIndex", 
            (unsigned int(VesselNode2::*)()) &VesselNode2::GetLocalIndex, 
            " "  )
        .def(
            "GetOwnerRank", 
            (unsigned int(VesselNode2::*)()) &VesselNode2::GetOwnerRank, 
            " "  )
        .def(
            "IsHalo", 
            (bool(VesselNode2::*)()) &VesselNode2::IsHalo, 
            " "  )
        .def(
            "HasHalo", 
            (bool(VesselNode2::*)()) &VesselNode2::HasHalo, 
            " "  )
        .def(
            "GetOtherProcessorRank", 
            (unsigned int(VesselNode2::*)()) &VesselNode2::GetOtherProcessorRank, 
            " "  )
        .def(
            "GetOtherProcessorLocalIndex", 
            (unsigned int(VesselNode2::*)()) &VesselNode2::GetOtherProcessorLocalIndex, 
            " "  )
        .def(
            "IsAttachedTo", 
            (bool(VesselNode2::*)(::std::shared_ptr<VesselSegment<2> > const) const ) &VesselNode2::IsAttachedTo, 
            " " , py::arg("pSegment") )
        .def(
            "IsCoincident", 
            (bool(VesselNode2::*)(::Vertex<2> const &) const ) &VesselNode2::IsCoincident, 
            " " , py::arg("rLocation") )
        .def(
            "IsMigrating", 
            (bool(VesselNode2::*)() const ) &VesselNode2::IsMigrating, 
            " "  )
        .def(
            "SetComparisonId", 
            (void(VesselNode2::*)(unsigned int)) &VesselNode2::SetComparisonId, 
            " " , py::arg("id") )
        .def(
            "SetFlowProperties", 
            (void(VesselNode2::*)(::NodeFlowProperties<2> const &)) &VesselNode2::SetFlowProperties, 
            " " , py::arg("rFlowProperties") )
        .def(
            "SetChemicalProperties", 
            (void(VesselNode2::*)(::NodeChemicalProperties<2> const &)) &VesselNode2::SetChemicalProperties, 
            " " , py::arg("rChemicalProperties") )
        .def(
            "SetIsMigrating", 
            (void(VesselNode2::*)(bool)) &VesselNode2::SetIsMigrating, 
            " " , py::arg("isMigrating") )
        .def(
            "SetLocation", 
            (void(VesselNode2::*)(::Vertex<2> const &)) &VesselNode2::SetLocation, 
            " " , py::arg("rLocation") )
        .def(
            "SetLocation", 
            (void(VesselNode2::*)(::QLength, ::QLength, ::QLength)) &VesselNode2::SetLocation, 
            " " , py::arg("v1"), py::arg("v2") = 0_m, py::arg("v3") = 0_m )
        .def(
            "SetGlobalIndex", 
            (void(VesselNode2::*)(unsigned int)) &VesselNode2::SetGlobalIndex, 
            " " , py::arg("index") )
        .def(
            "SetLocalIndex", 
            (void(VesselNode2::*)(unsigned int)) &VesselNode2::SetLocalIndex, 
            " " , py::arg("index") )
        .def(
            "SetOwnerRank", 
            (void(VesselNode2::*)(unsigned int)) &VesselNode2::SetOwnerRank, 
            " " , py::arg("rank") )
        .def(
            "SetIsHalo", 
            (void(VesselNode2::*)(bool)) &VesselNode2::SetIsHalo, 
            " " , py::arg("isHalo") )
        .def(
            "SetHasHalo", 
            (void(VesselNode2::*)(bool)) &VesselNode2::SetHasHalo, 
            " " , py::arg("hasHalo") )
        .def(
            "SetOtherProcessorRank", 
            (void(VesselNode2::*)(unsigned int)) &VesselNode2::SetOtherProcessorRank, 
            " " , py::arg("otherRank") )
        .def(
            "SetOtherProcessorLocalIndex", 
            (void(VesselNode2::*)(unsigned int)) &VesselNode2::SetOtherProcessorLocalIndex, 
            " " , py::arg("otherIndex") )
    ;
}
