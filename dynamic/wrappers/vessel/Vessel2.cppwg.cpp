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
#include "Vessel.hpp"

#include "Vessel2.cppwg.hpp"

namespace py = pybind11;
typedef Vessel<2 > Vessel2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::QLength _QLength;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class Vessel2_Overloads : public Vessel2{
    public:
    using Vessel2::Vessel;
    ::QLength GetRadius() const  override {
        PYBIND11_OVERLOAD(
            _QLength,
            Vessel2,
            GetRadius,
            );
    }
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            Vessel2,
            GetOutputData,
            );
    }
    void SetRadius(::QLength radius) override {
        PYBIND11_OVERLOAD(
            void,
            Vessel2,
            SetRadius,
            radius);
    }

};
void register_Vessel2_class(py::module &m){
py::class_<Vessel2 , Vessel2_Overloads , std::shared_ptr<Vessel2 >   >(m, "Vessel2")
        .def_static(
            "Create", 
            (::std::shared_ptr<Vessel<2> >(*)(::std::shared_ptr<VesselSegment<2> >)) &Vessel2::Create, 
            " " , py::arg("pSegment") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vessel<2> >(*)(::std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >)) &Vessel2::Create, 
            " " , py::arg("segments") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vessel<2> >(*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >)) &Vessel2::Create, 
            " " , py::arg("nodes") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Vessel<2> >(*)(::std::shared_ptr<VesselNode<2> >, ::std::shared_ptr<VesselNode<2> >)) &Vessel2::Create, 
            " " , py::arg("pStartNode"), py::arg("pEndNode") )
        .def(
            "AddSegment", 
            (void(Vessel2::*)(::std::shared_ptr<VesselSegment<2> >)) &Vessel2::AddSegment, 
            " " , py::arg("pSegment") )
        .def(
            "AddSegments", 
            (void(Vessel2::*)(::std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >)) &Vessel2::AddSegments, 
            " " , py::arg("pSegments") )
        .def(
            "CopyDataFromExistingVessel", 
            (void(Vessel2::*)(::std::shared_ptr<Vessel<2> >)) &Vessel2::CopyDataFromExistingVessel, 
            " " , py::arg("pTargetVessel") )
        .def(
            "DivideSegment", 
            (::std::shared_ptr<VesselNode<2> >(Vessel2::*)(::Vertex<2> const &, ::QLength)) &Vessel2::DivideSegment, 
            " " , py::arg("rLocation"), py::arg("distanceTolerance") = 1.0E-12_m )
        .def(
            "GetClosestEndNodeDistance", 
            (::QLength(Vessel2::*)(::Vertex<2> const &)) &Vessel2::GetClosestEndNodeDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetDistance", 
            (::QLength(Vessel2::*)(::Vertex<2> const &) const ) &Vessel2::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetConnectedVessels", 
            (::std::vector<std::shared_ptr<Vessel<2> >, std::allocator<std::shared_ptr<Vessel<2> > > >(Vessel2::*)()) &Vessel2::GetConnectedVessels, 
            " "  )
        .def(
            "GetEndNode", 
            (::std::shared_ptr<VesselNode<2> >(Vessel2::*)()) &Vessel2::GetEndNode, 
            " "  )
        .def(
            "GetFlowProperties", 
            (::std::shared_ptr<VesselFlowProperties<2> >(Vessel2::*)() const ) &Vessel2::GetFlowProperties, 
            " "  )
        .def(
            "GetNodeAtOppositeEnd", 
            (::std::shared_ptr<VesselNode<2> >(Vessel2::*)(::std::shared_ptr<VesselNode<2> >)) &Vessel2::GetNodeAtOppositeEnd, 
            " " , py::arg("pQueryNode") )
        .def(
            "GetLength", 
            (::QLength(Vessel2::*)() const ) &Vessel2::GetLength, 
            " "  )
        .def(
            "GetRadius", 
            (::QLength(Vessel2::*)() const ) &Vessel2::GetRadius, 
            " "  )
        .def(
            "GetMaturity", 
            (double(Vessel2::*)() const ) &Vessel2::GetMaturity, 
            " "  )
        .def(
            "GetNode", 
            (::std::shared_ptr<VesselNode<2> >(Vessel2::*)(unsigned int)) &Vessel2::GetNode, 
            " " , py::arg("index") )
        .def(
            "GetNodes", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(Vessel2::*)()) &Vessel2::GetNodes, 
            " "  )
        .def(
            "rGetNodes", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &(Vessel2::*)()) &Vessel2::rGetNodes, 
            " "  )
        .def(
            "GetNumberOfNodes", 
            (unsigned int(Vessel2::*)()) &Vessel2::GetNumberOfNodes, 
            " "  )
        .def(
            "GetNumberOfSegments", 
            (unsigned int(Vessel2::*)()) &Vessel2::GetNumberOfSegments, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Vessel2::*)()) &Vessel2::GetOutputData, 
            " "  )
        .def(
            "GetSegment", 
            (::std::shared_ptr<VesselSegment<2> >(Vessel2::*)(unsigned int)) &Vessel2::GetSegment, 
            " " , py::arg("index") )
        .def(
            "GetSegments", 
            (::std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >(Vessel2::*)()) &Vessel2::GetSegments, 
            " "  )
        .def(
            "GetStartNode", 
            (::std::shared_ptr<VesselNode<2> >(Vessel2::*)()) &Vessel2::GetStartNode, 
            " "  )
        .def(
            "GetGlobalIndex", 
            (unsigned int(Vessel2::*)()) &Vessel2::GetGlobalIndex, 
            " "  )
        .def(
            "GetLocalIndex", 
            (unsigned int(Vessel2::*)()) &Vessel2::GetLocalIndex, 
            " "  )
        .def(
            "GetOwnerRank", 
            (unsigned int(Vessel2::*)()) &Vessel2::GetOwnerRank, 
            " "  )
        .def(
            "IsHalo", 
            (bool(Vessel2::*)()) &Vessel2::IsHalo, 
            " "  )
        .def(
            "HasHalo", 
            (bool(Vessel2::*)()) &Vessel2::HasHalo, 
            " "  )
        .def(
            "GetOtherProcessorRank", 
            (unsigned int(Vessel2::*)()) &Vessel2::GetOtherProcessorRank, 
            " "  )
        .def(
            "GetOtherProcessorLocalIndex", 
            (unsigned int(Vessel2::*)()) &Vessel2::GetOtherProcessorLocalIndex, 
            " "  )
        .def(
            "IsConnectedTo", 
            (bool(Vessel2::*)(::std::shared_ptr<Vessel<2> >)) &Vessel2::IsConnectedTo, 
            " " , py::arg("pOtherVessel") )
        .def(
            "Remove", 
            (void(Vessel2::*)()) &Vessel2::Remove, 
            " "  )
        .def(
            "RemoveSegments", 
            (void(Vessel2::*)(::SegmentLocation::Value)) &Vessel2::RemoveSegments, 
            " " , py::arg("location") )
        .def(
            "SetRadius", 
            (void(Vessel2::*)(::QLength)) &Vessel2::SetRadius, 
            " " , py::arg("radius") )
        .def(
            "SetFlowProperties", 
            (void(Vessel2::*)(::VesselFlowProperties<2> const &)) &Vessel2::SetFlowProperties, 
            " " , py::arg("rFlowProperties") )
        .def(
            "SetGlobalIndex", 
            (void(Vessel2::*)(unsigned int)) &Vessel2::SetGlobalIndex, 
            " " , py::arg("index") )
        .def(
            "SetLocalIndex", 
            (void(Vessel2::*)(unsigned int)) &Vessel2::SetLocalIndex, 
            " " , py::arg("index") )
        .def(
            "SetOwnerRank", 
            (void(Vessel2::*)(unsigned int)) &Vessel2::SetOwnerRank, 
            " " , py::arg("rank") )
        .def(
            "SetIsHalo", 
            (void(Vessel2::*)(bool)) &Vessel2::SetIsHalo, 
            " " , py::arg("isHalo") )
        .def(
            "SetHasHalo", 
            (void(Vessel2::*)(bool)) &Vessel2::SetHasHalo, 
            " " , py::arg("hasHalo") )
        .def(
            "SetOtherProcessorRank", 
            (void(Vessel2::*)(unsigned int)) &Vessel2::SetOtherProcessorRank, 
            " " , py::arg("otherRank") )
        .def(
            "SetOtherProcessorLocalIndex", 
            (void(Vessel2::*)(unsigned int)) &Vessel2::SetOtherProcessorLocalIndex, 
            " " , py::arg("otherIndex") )
        .def(
            "UpdateNodes", 
            (void(Vessel2::*)()) &Vessel2::UpdateNodes, 
            " "  )
    ;
}
