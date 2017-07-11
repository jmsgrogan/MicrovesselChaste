#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselNode.hpp"

#include "VesselNode3.cppwg.hpp"

namespace py = pybind11;
typedef VesselNode<3 > VesselNode3;
;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class VesselNode3_Overloads : public VesselNode3{
    public:
    using VesselNode3::VesselNode;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            VesselNode3,
            GetOutputData,
            );
    }

};
void register_VesselNode3_class(py::module &m){
py::class_<VesselNode3 , VesselNode3_Overloads   >(m, "VesselNode3")
        .def(py::init<double, double, double, ::QLength >(), py::arg("v1"), py::arg("v2"), py::arg("v3"), py::arg("referenceLength"))
        .def(py::init<double, double, double >(), py::arg("v1") = 0., py::arg("v2") = 0., py::arg("v3") = 0.)
        .def(py::init<::Vertex<3> const & >(), py::arg("location"))
        .def(py::init<::VesselNode<3> const & >(), py::arg("rExistingNode"))
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<3> >(*)(double, double, double)) &VesselNode3::Create, 
            " " , py::arg("v1") = 0., py::arg("v2") = 0., py::arg("v3") = 0. )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<3> >(*)(double, double, double, ::QLength)) &VesselNode3::Create, 
            " " , py::arg("v1"), py::arg("v2"), py::arg("v3"), py::arg("referenceLength") )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<3> >(*)(::Vertex<3> const &)) &VesselNode3::Create, 
            " " , py::arg("location") )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<3> >(*)(::VesselNode<3> const &)) &VesselNode3::Create, 
            " " , py::arg("rExistingNode") )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNode<3> >(*)(::std::shared_ptr<VesselNode<3> >)) &VesselNode3::Create, 
            " " , py::arg("pExistingNode") )
        .def(
            "GetComparisonId", 
            (unsigned int(VesselNode3::*)()) &VesselNode3::GetComparisonId, 
            " "  )
        .def(
            "GetDistance", 
            (::QLength(VesselNode3::*)(::Vertex<3> const &) const ) &VesselNode3::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetFlowProperties", 
            (::std::shared_ptr<NodeFlowProperties<3> >(VesselNode3::*)() const ) &VesselNode3::GetFlowProperties, 
            " "  )
        .def(
            "rGetLocation", 
            (::Vertex<3> const &(VesselNode3::*)() const ) &VesselNode3::rGetLocation, 
            " "  )
        .def(
            "GetNumberOfSegments", 
            (unsigned int(VesselNode3::*)() const ) &VesselNode3::GetNumberOfSegments, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(VesselNode3::*)()) &VesselNode3::GetOutputData, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(VesselNode3::*)() const ) &VesselNode3::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetSegment", 
            (::std::shared_ptr<VesselSegment<3> >(VesselNode3::*)(unsigned int) const ) &VesselNode3::GetSegment, 
            " " , py::arg("index") )
        .def(
            "GetSegments", 
            (::std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >(VesselNode3::*)() const ) &VesselNode3::GetSegments, 
            " "  )
        .def(
            "GetGlobalIndex", 
            (unsigned int(VesselNode3::*)()) &VesselNode3::GetGlobalIndex, 
            " "  )
        .def(
            "GetLocalIndex", 
            (unsigned int(VesselNode3::*)()) &VesselNode3::GetLocalIndex, 
            " "  )
        .def(
            "GetOwnerRank", 
            (unsigned int(VesselNode3::*)()) &VesselNode3::GetOwnerRank, 
            " "  )
        .def(
            "IsHalo", 
            (bool(VesselNode3::*)()) &VesselNode3::IsHalo, 
            " "  )
        .def(
            "HasHalo", 
            (bool(VesselNode3::*)()) &VesselNode3::HasHalo, 
            " "  )
        .def(
            "GetOtherProcessorRank", 
            (unsigned int(VesselNode3::*)()) &VesselNode3::GetOtherProcessorRank, 
            " "  )
        .def(
            "GetOtherProcessorLocalIndex", 
            (unsigned int(VesselNode3::*)()) &VesselNode3::GetOtherProcessorLocalIndex, 
            " "  )
        .def(
            "IsAttachedTo", 
            (bool(VesselNode3::*)(::std::shared_ptr<VesselSegment<3> > const) const ) &VesselNode3::IsAttachedTo, 
            " " , py::arg("pSegment") )
        .def(
            "IsCoincident", 
            (bool(VesselNode3::*)(::Vertex<3> const &) const ) &VesselNode3::IsCoincident, 
            " " , py::arg("rLocation") )
        .def(
            "IsMigrating", 
            (bool(VesselNode3::*)() const ) &VesselNode3::IsMigrating, 
            " "  )
        .def(
            "SetComparisonId", 
            (void(VesselNode3::*)(unsigned int)) &VesselNode3::SetComparisonId, 
            " " , py::arg("id") )
        .def(
            "SetFlowProperties", 
            (void(VesselNode3::*)(::NodeFlowProperties<3> const &)) &VesselNode3::SetFlowProperties, 
            " " , py::arg("rFlowProperties") )
        .def(
            "SetIsMigrating", 
            (void(VesselNode3::*)(bool)) &VesselNode3::SetIsMigrating, 
            " " , py::arg("isMigrating") )
        .def(
            "SetLocation", 
            (void(VesselNode3::*)(::Vertex<3> const &)) &VesselNode3::SetLocation, 
            " " , py::arg("rLocation") )
        .def(
            "SetLocation", 
            (void(VesselNode3::*)(double, double, double, ::QLength)) &VesselNode3::SetLocation, 
            " " , py::arg("x"), py::arg("y"), py::arg("z") = 0., py::arg("referenceLength") = 9.9999999999999995E-7 * unit::metres )
        .def(
            "SetReferenceLengthScale", 
            (void(VesselNode3::*)(::QLength)) &VesselNode3::SetReferenceLengthScale, 
            " " , py::arg("lenthScale") )
        .def(
            "SetGlobalIndex", 
            (void(VesselNode3::*)(unsigned int)) &VesselNode3::SetGlobalIndex, 
            " " , py::arg("index") )
        .def(
            "SetLocalIndex", 
            (void(VesselNode3::*)(unsigned int)) &VesselNode3::SetLocalIndex, 
            " " , py::arg("index") )
        .def(
            "SetOwnerRank", 
            (void(VesselNode3::*)(unsigned int)) &VesselNode3::SetOwnerRank, 
            " " , py::arg("rank") )
        .def(
            "SetIsHalo", 
            (void(VesselNode3::*)(bool)) &VesselNode3::SetIsHalo, 
            " " , py::arg("isHalo") )
        .def(
            "SetHasHalo", 
            (void(VesselNode3::*)(bool)) &VesselNode3::SetHasHalo, 
            " " , py::arg("hasHalo") )
        .def(
            "SetOtherProcessorRank", 
            (void(VesselNode3::*)(unsigned int)) &VesselNode3::SetOtherProcessorRank, 
            " " , py::arg("otherRank") )
        .def(
            "SetOtherProcessorLocalIndex", 
            (void(VesselNode3::*)(unsigned int)) &VesselNode3::SetOtherProcessorLocalIndex, 
            " " , py::arg("otherIndex") )
    ;
}
