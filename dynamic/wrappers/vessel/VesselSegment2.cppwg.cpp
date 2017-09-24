#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "VesselNode.hpp"
#include "Vessel.hpp"
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "vtkPolyData.h"
#include "VesselSegment.hpp"

#include "PythonObjectConverters.hpp"
#include "VesselSegment2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VesselSegment<2 > VesselSegment2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_;

class VesselSegment2_Overloads : public VesselSegment2{
    public:
    using VesselSegment2::VesselSegment;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD(
            _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_,
            VesselSegment2,
            GetOutputData,
            );
    }

};
void register_VesselSegment2_class(py::module &m){
py::class_<VesselSegment2 , VesselSegment2_Overloads , std::shared_ptr<VesselSegment2 >  , AbstractVesselNetworkComponent<2>  >(m, "VesselSegment2")
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselSegment<2> >(*)(::std::shared_ptr<VesselNode<2> >, ::std::shared_ptr<VesselNode<2> >)) &VesselSegment2::Create, 
            " " , py::arg("pNode1"), py::arg("pNode2") )
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselSegment<2> >(*)(::std::shared_ptr<VesselSegment<2> >)) &VesselSegment2::Create, 
            " " , py::arg("pSegment") )
        .def(
            "CopyDataFromExistingSegment", 
            (void(VesselSegment2::*)(::std::shared_ptr<VesselSegment<2> > const)) &VesselSegment2::CopyDataFromExistingSegment, 
            " " , py::arg("pTargetSegment") )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(VesselSegment2::*)()) &VesselSegment2::GetOutputData, 
            " "  )
        .def(
            "GetDistance", 
            (::QLength(VesselSegment2::*)(::Vertex<2> const &) const ) &VesselSegment2::GetDistance, 
            " " , py::arg("location") )
        .def(
            "GetFlowProperties", 
            (::std::shared_ptr<SegmentFlowProperties<2> >(VesselSegment2::*)() const ) &VesselSegment2::GetFlowProperties, 
            " "  )
        .def(
            "GetCellularProperties", 
            (::std::shared_ptr<SegmentCellularProperties<2> >(VesselSegment2::*)() const ) &VesselSegment2::GetCellularProperties, 
            " "  )
        .def(
            "GetLength", 
            (::QLength(VesselSegment2::*)() const ) &VesselSegment2::GetLength, 
            " "  )
        .def(
            "GetMidPoint", 
            (::Vertex<2>(VesselSegment2::*)() const ) &VesselSegment2::GetMidPoint, 
            " "  )
        .def(
            "GetMaturity", 
            (double(VesselSegment2::*)() const ) &VesselSegment2::GetMaturity, 
            " "  )
        .def(
            "GetGlobalIndex", 
            (unsigned int(VesselSegment2::*)()) &VesselSegment2::GetGlobalIndex, 
            " "  )
        .def(
            "GetLocalIndex", 
            (unsigned int(VesselSegment2::*)()) &VesselSegment2::GetLocalIndex, 
            " "  )
        .def(
            "GetOwnerRank", 
            (unsigned int(VesselSegment2::*)()) &VesselSegment2::GetOwnerRank, 
            " "  )
        .def(
            "IsHalo", 
            (bool(VesselSegment2::*)()) &VesselSegment2::IsHalo, 
            " "  )
        .def(
            "HasHalo", 
            (bool(VesselSegment2::*)()) &VesselSegment2::HasHalo, 
            " "  )
        .def(
            "GetOtherProcessorRank", 
            (unsigned int(VesselSegment2::*)()) &VesselSegment2::GetOtherProcessorRank, 
            " "  )
        .def(
            "GetOtherProcessorLocalIndex", 
            (unsigned int(VesselSegment2::*)()) &VesselSegment2::GetOtherProcessorLocalIndex, 
            " "  )
        .def(
            "GetNode", 
            (::std::shared_ptr<VesselNode<2> >(VesselSegment2::*)(unsigned int) const ) &VesselSegment2::GetNode, 
            " " , py::arg("index") )
        .def(
            "GetOppositeNode", 
            (::std::shared_ptr<VesselNode<2> >(VesselSegment2::*)(::std::shared_ptr<VesselNode<2> >) const ) &VesselSegment2::GetOppositeNode, 
            " " , py::arg("pInputNode") )
        .def(
            "GetNodes", 
            (::std::pair<std::shared_ptr<VesselNode<2> >, std::shared_ptr<VesselNode<2> > >(VesselSegment2::*)() const ) &VesselSegment2::GetNodes, 
            " "  )
        .def(
            "GetPointProjection", 
            (::Vertex<2>(VesselSegment2::*)(::Vertex<2> const &, bool) const ) &VesselSegment2::GetPointProjection, 
            " " , py::arg("location"), py::arg("projectToEnds") = false )
        .def(
            "GetUnitTangent", 
            (::boost::numeric::ublas::c_vector<double, 2>(VesselSegment2::*)() const ) &VesselSegment2::GetUnitTangent, 
            " "  )
        .def(
            "GetVessel", 
            (::std::shared_ptr<Vessel<2> >(VesselSegment2::*)() const ) &VesselSegment2::GetVessel, 
            " "  )
        .def(
            "HasNode", 
            (bool(VesselSegment2::*)(::std::shared_ptr<VesselNode<2> >) const ) &VesselSegment2::HasNode, 
            " " , py::arg("pNode") )
        .def(
            "IsConnectedTo", 
            (bool(VesselSegment2::*)(::std::shared_ptr<VesselSegment<2> >) const ) &VesselSegment2::IsConnectedTo, 
            " " , py::arg("pOtherSegment") )
        .def(
            "ReplaceNode", 
            (void(VesselSegment2::*)(unsigned int, ::std::shared_ptr<VesselNode<2> >)) &VesselSegment2::ReplaceNode, 
            " " , py::arg("oldNodeIndex"), py::arg("pNewNode") )
        .def(
            "Remove", 
            (void(VesselSegment2::*)()) &VesselSegment2::Remove, 
            " "  )
        .def(
            "SetFlowProperties", 
            (void(VesselSegment2::*)(::SegmentFlowProperties<2> const &)) &VesselSegment2::SetFlowProperties, 
            " " , py::arg("rFlowProperties") )
        .def(
            "SetCellularProperties", 
            (void(VesselSegment2::*)(::SegmentCellularProperties<2> const &)) &VesselSegment2::SetCellularProperties, 
            " " , py::arg("rCellularProperties") )
        .def(
            "SetMaturity", 
            (void(VesselSegment2::*)(double)) &VesselSegment2::SetMaturity, 
            " " , py::arg("maturity") )
        .def(
            "SetGlobalIndex", 
            (void(VesselSegment2::*)(unsigned int)) &VesselSegment2::SetGlobalIndex, 
            " " , py::arg("index") )
        .def(
            "SetLocalIndex", 
            (void(VesselSegment2::*)(unsigned int)) &VesselSegment2::SetLocalIndex, 
            " " , py::arg("index") )
        .def(
            "SetOwnerRank", 
            (void(VesselSegment2::*)(unsigned int)) &VesselSegment2::SetOwnerRank, 
            " " , py::arg("rank") )
        .def(
            "SetIsHalo", 
            (void(VesselSegment2::*)(bool)) &VesselSegment2::SetIsHalo, 
            " " , py::arg("isHalo") )
        .def(
            "SetHasHalo", 
            (void(VesselSegment2::*)(bool)) &VesselSegment2::SetHasHalo, 
            " " , py::arg("hasHalo") )
        .def(
            "SetOtherProcessorRank", 
            (void(VesselSegment2::*)(unsigned int)) &VesselSegment2::SetOtherProcessorRank, 
            " " , py::arg("otherRank") )
        .def(
            "SetOtherProcessorLocalIndex", 
            (void(VesselSegment2::*)(unsigned int)) &VesselSegment2::SetOtherProcessorLocalIndex, 
            " " , py::arg("otherIndex") )
    ;
}
