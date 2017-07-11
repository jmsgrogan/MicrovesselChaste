#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselSegment.hpp"

#include "VesselSegment3.cppwg.hpp"

namespace py = pybind11;
typedef VesselSegment<3 > VesselSegment3;
;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class VesselSegment3_Overloads : public VesselSegment3{
    public:
    using VesselSegment3::VesselSegment;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            VesselSegment3,
            GetOutputData,
            );
    }

};
void register_VesselSegment3_class(py::module &m){
py::class_<VesselSegment3 , VesselSegment3_Overloads   >(m, "VesselSegment3")
        .def(py::init<::VesselSegment<3> const & >(), py::arg("rSegment"))
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselSegment<3> >(*)(::std::shared_ptr<VesselNode<3> >, ::std::shared_ptr<VesselNode<3> >)) &VesselSegment3::Create, 
            " " , py::arg("pNode1"), py::arg("pNode2") )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselSegment<3> >(*)(::std::shared_ptr<VesselSegment<3> >)) &VesselSegment3::Create, 
            " " , py::arg("pSegment") )
        .def(
            "CopyDataFromExistingSegment", 
            (void(VesselSegment3::*)(::std::shared_ptr<VesselSegment<3> > const)) &VesselSegment3::CopyDataFromExistingSegment, 
            " " , py::arg("pTargetSegment") )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(VesselSegment3::*)()) &VesselSegment3::GetOutputData, 
            " "  )
        .def(
            "GetDistance", 
            (::QLength(VesselSegment3::*)(::Vertex<3> const &) const ) &VesselSegment3::GetDistance, 
            " " , py::arg("location") )
        .def(
            "GetFlowProperties", 
            (::std::shared_ptr<SegmentFlowProperties<3> >(VesselSegment3::*)() const ) &VesselSegment3::GetFlowProperties, 
            " "  )
        .def(
            "GetLength", 
            (::QLength(VesselSegment3::*)() const ) &VesselSegment3::GetLength, 
            " "  )
        .def(
            "GetMidPoint", 
            (::Vertex<3>(VesselSegment3::*)() const ) &VesselSegment3::GetMidPoint, 
            " "  )
        .def(
            "GetMaturity", 
            (double(VesselSegment3::*)() const ) &VesselSegment3::GetMaturity, 
            " "  )
        .def(
            "GetGlobalIndex", 
            (unsigned int(VesselSegment3::*)()) &VesselSegment3::GetGlobalIndex, 
            " "  )
        .def(
            "GetLocalIndex", 
            (unsigned int(VesselSegment3::*)()) &VesselSegment3::GetLocalIndex, 
            " "  )
        .def(
            "GetOwnerRank", 
            (unsigned int(VesselSegment3::*)()) &VesselSegment3::GetOwnerRank, 
            " "  )
        .def(
            "IsHalo", 
            (bool(VesselSegment3::*)()) &VesselSegment3::IsHalo, 
            " "  )
        .def(
            "HasHalo", 
            (bool(VesselSegment3::*)()) &VesselSegment3::HasHalo, 
            " "  )
        .def(
            "GetOtherProcessorRank", 
            (unsigned int(VesselSegment3::*)()) &VesselSegment3::GetOtherProcessorRank, 
            " "  )
        .def(
            "GetOtherProcessorLocalIndex", 
            (unsigned int(VesselSegment3::*)()) &VesselSegment3::GetOtherProcessorLocalIndex, 
            " "  )
        .def(
            "GetNode", 
            (::std::shared_ptr<VesselNode<3> >(VesselSegment3::*)(unsigned int) const ) &VesselSegment3::GetNode, 
            " " , py::arg("index") )
        .def(
            "GetOppositeNode", 
            (::std::shared_ptr<VesselNode<3> >(VesselSegment3::*)(::std::shared_ptr<VesselNode<3> >) const ) &VesselSegment3::GetOppositeNode, 
            " " , py::arg("pInputNode") )
        .def(
            "GetNodes", 
            (::std::pair<std::shared_ptr<VesselNode<3> >, std::shared_ptr<VesselNode<3> > >(VesselSegment3::*)() const ) &VesselSegment3::GetNodes, 
            " "  )
        .def(
            "GetPointProjection", 
            (::Vertex<3>(VesselSegment3::*)(::Vertex<3> const &, bool) const ) &VesselSegment3::GetPointProjection, 
            " " , py::arg("location"), py::arg("projectToEnds") = false )
        .def(
            "GetUnitTangent", 
            (::boost::numeric::ublas::c_vector<double, 3>(VesselSegment3::*)() const ) &VesselSegment3::GetUnitTangent, 
            " "  )
        .def(
            "GetVessel", 
            (::std::shared_ptr<Vessel<3> >(VesselSegment3::*)() const ) &VesselSegment3::GetVessel, 
            " "  )
        .def(
            "HasNode", 
            (bool(VesselSegment3::*)(::std::shared_ptr<VesselNode<3> >) const ) &VesselSegment3::HasNode, 
            " " , py::arg("pNode") )
        .def(
            "IsConnectedTo", 
            (bool(VesselSegment3::*)(::std::shared_ptr<VesselSegment<3> >) const ) &VesselSegment3::IsConnectedTo, 
            " " , py::arg("pOtherSegment") )
        .def(
            "ReplaceNode", 
            (void(VesselSegment3::*)(unsigned int, ::std::shared_ptr<VesselNode<3> >)) &VesselSegment3::ReplaceNode, 
            " " , py::arg("oldNodeIndex"), py::arg("pNewNode") )
        .def(
            "Remove", 
            (void(VesselSegment3::*)()) &VesselSegment3::Remove, 
            " "  )
        .def(
            "SetFlowProperties", 
            (void(VesselSegment3::*)(::SegmentFlowProperties<3> const &)) &VesselSegment3::SetFlowProperties, 
            " " , py::arg("rFlowProperties") )
        .def(
            "SetMaturity", 
            (void(VesselSegment3::*)(double)) &VesselSegment3::SetMaturity, 
            " " , py::arg("maturity") )
        .def(
            "SetGlobalIndex", 
            (void(VesselSegment3::*)(unsigned int)) &VesselSegment3::SetGlobalIndex, 
            " " , py::arg("index") )
        .def(
            "SetLocalIndex", 
            (void(VesselSegment3::*)(unsigned int)) &VesselSegment3::SetLocalIndex, 
            " " , py::arg("index") )
        .def(
            "SetOwnerRank", 
            (void(VesselSegment3::*)(unsigned int)) &VesselSegment3::SetOwnerRank, 
            " " , py::arg("rank") )
        .def(
            "SetIsHalo", 
            (void(VesselSegment3::*)(bool)) &VesselSegment3::SetIsHalo, 
            " " , py::arg("isHalo") )
        .def(
            "SetHasHalo", 
            (void(VesselSegment3::*)(bool)) &VesselSegment3::SetHasHalo, 
            " " , py::arg("hasHalo") )
        .def(
            "SetOtherProcessorRank", 
            (void(VesselSegment3::*)(unsigned int)) &VesselSegment3::SetOtherProcessorRank, 
            " " , py::arg("otherRank") )
        .def(
            "SetOtherProcessorLocalIndex", 
            (void(VesselSegment3::*)(unsigned int)) &VesselSegment3::SetOtherProcessorLocalIndex, 
            " " , py::arg("otherIndex") )
    ;
}
