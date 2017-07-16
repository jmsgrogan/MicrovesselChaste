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
#include "VesselNetwork.hpp"

#include "VesselNetwork2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetwork<2 > VesselNetwork2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::shared_ptr<VesselNode<2> > _std_shared_ptr_lt_VesselNode_lt_2_gt__gt_;
typedef ::std::shared_ptr<Vessel<2> > _std_shared_ptr_lt_Vessel_lt_2_gt__gt_;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_;

class VesselNetwork2_Overloads : public VesselNetwork2{
    public:
    using VesselNetwork2::VesselNetwork;
    ::std::shared_ptr<VesselNode<2> > DivideVessel(::std::shared_ptr<Vessel<2> > pVessel, ::Vertex<2> const & rLocation) override {
        PYBIND11_OVERLOAD(
            _std_shared_ptr_lt_VesselNode_lt_2_gt__gt_,
            VesselNetwork2,
            DivideVessel,
            pVessel, 
rLocation);
    }
    void ExtendVessel(::std::shared_ptr<Vessel<2> > pVessel, ::std::shared_ptr<VesselNode<2> > pEndNode, ::std::shared_ptr<VesselNode<2> > pNewNode) override {
        PYBIND11_OVERLOAD(
            void,
            VesselNetwork2,
            ExtendVessel,
            pVessel, 
pEndNode, 
pNewNode);
    }
    ::std::shared_ptr<Vessel<2> > FormSprout(::std::shared_ptr<VesselNode<2> > pSproutBase, ::Vertex<2> const & sproutTipLocation) override {
        PYBIND11_OVERLOAD(
            _std_shared_ptr_lt_Vessel_lt_2_gt__gt_,
            VesselNetwork2,
            FormSprout,
            pSproutBase, 
sproutTipLocation);
    }
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD(
            _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_,
            VesselNetwork2,
            GetOutputData,
            );
    }

};
void register_VesselNetwork2_class(py::module &m){
py::class_<VesselNetwork2 , VesselNetwork2_Overloads , std::shared_ptr<VesselNetwork2 >  , AbstractVesselNetworkComponent<2>  >(m, "VesselNetwork2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetwork<2> >(*)()) &VesselNetwork2::Create, 
            " "  )
        .def(
            "AddVessel", 
            (void(VesselNetwork2::*)(::std::shared_ptr<Vessel<2> >)) &VesselNetwork2::AddVessel, 
            " " , py::arg("pVessel") )
        .def(
            "AddVessels", 
            (void(VesselNetwork2::*)(::std::vector<std::shared_ptr<Vessel<2> >, std::allocator<std::shared_ptr<Vessel<2> > > >)) &VesselNetwork2::AddVessels, 
            " " , py::arg("vessels") )
        .def(
            "ClearVessels", 
            (void(VesselNetwork2::*)()) &VesselNetwork2::ClearVessels, 
            " "  )
        .def(
            "CopyVessels", 
            (::std::vector<std::shared_ptr<Vessel<2> >, std::allocator<std::shared_ptr<Vessel<2> > > >(VesselNetwork2::*)()) &VesselNetwork2::CopyVessels, 
            " "  )
        .def(
            "CopyVessels", 
            (::std::vector<std::shared_ptr<Vessel<2> >, std::allocator<std::shared_ptr<Vessel<2> > > >(VesselNetwork2::*)(::std::vector<std::shared_ptr<Vessel<2> >, std::allocator<std::shared_ptr<Vessel<2> > > >)) &VesselNetwork2::CopyVessels, 
            " " , py::arg("vessels") )
        .def(
            "DivideVessel", 
            (::std::shared_ptr<VesselNode<2> >(VesselNetwork2::*)(::std::shared_ptr<Vessel<2> >, ::Vertex<2> const &)) &VesselNetwork2::DivideVessel, 
            " " , py::arg("pVessel"), py::arg("rLocation") )
        .def(
            "ExtendVessel", 
            (void(VesselNetwork2::*)(::std::shared_ptr<Vessel<2> >, ::std::shared_ptr<VesselNode<2> >, ::std::shared_ptr<VesselNode<2> >)) &VesselNetwork2::ExtendVessel, 
            " " , py::arg("pVessel"), py::arg("pEndNode"), py::arg("pNewNode") )
        .def(
            "FormSprout", 
            (::std::shared_ptr<Vessel<2> >(VesselNetwork2::*)(::std::shared_ptr<VesselNode<2> >, ::Vertex<2> const &)) &VesselNetwork2::FormSprout, 
            " " , py::arg("pSproutBase"), py::arg("sproutTipLocation") )
        .def(
            "GetNodeIndex", 
            (unsigned int(VesselNetwork2::*)(::std::shared_ptr<VesselNode<2> >)) &VesselNetwork2::GetNodeIndex, 
            " " , py::arg("pNode") )
        .def(
            "SetDistributedVectorFactory", 
            (void(VesselNetwork2::*)(::std::shared_ptr<DistributedVectorFactory>)) &VesselNetwork2::SetDistributedVectorFactory, 
            " " , py::arg("vectorFactory") )
        .def(
            "GetDistributedVectorFactory", 
            (::std::shared_ptr<DistributedVectorFactory>(VesselNetwork2::*)()) &VesselNetwork2::GetDistributedVectorFactory, 
            " "  )
        .def(
            "GetNode", 
            (::std::shared_ptr<VesselNode<2> >(VesselNetwork2::*)(unsigned int)) &VesselNetwork2::GetNode, 
            " " , py::arg("index") )
        .def(
            "GetNodes", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(VesselNetwork2::*)()) &VesselNetwork2::GetNodes, 
            " "  )
        .def(
            "GetVtk", 
            (::vtkSmartPointer<vtkPolyData>(VesselNetwork2::*)()) &VesselNetwork2::GetVtk, 
            " "  )
        .def(
            "GetVtkCellLocator", 
            (::vtkSmartPointer<vtkCellLocator>(VesselNetwork2::*)()) &VesselNetwork2::GetVtkCellLocator, 
            " "  )
        .def(
            "GetNumberOfNodes", 
            (unsigned int(VesselNetwork2::*)()) &VesselNetwork2::GetNumberOfNodes, 
            " "  )
        .def(
            "GetNumberOfNodesPerProcess", 
            (::std::vector<unsigned int, std::allocator<unsigned int> >(VesselNetwork2::*)()) &VesselNetwork2::GetNumberOfNodesPerProcess, 
            " "  )
        .def(
            "GetNumberOfVesselNodes", 
            (unsigned int(VesselNetwork2::*)()) &VesselNetwork2::GetNumberOfVesselNodes, 
            " "  )
        .def(
            "GetNumberOfVessels", 
            (unsigned int(VesselNetwork2::*)()) &VesselNetwork2::GetNumberOfVessels, 
            " "  )
        .def(
            "GetMaxBranchesOnNode", 
            (unsigned int(VesselNetwork2::*)()) &VesselNetwork2::GetMaxBranchesOnNode, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(VesselNetwork2::*)()) &VesselNetwork2::GetOutputData, 
            " "  )
        .def(
            "GetVesselEndNodes", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(VesselNetwork2::*)()) &VesselNetwork2::GetVesselEndNodes, 
            " "  )
        .def(
            "GetVesselIndex", 
            (unsigned int(VesselNetwork2::*)(::std::shared_ptr<Vessel<2> >)) &VesselNetwork2::GetVesselIndex, 
            " " , py::arg("pVessel") )
        .def(
            "GetVesselSegmentIndex", 
            (unsigned int(VesselNetwork2::*)(::std::shared_ptr<VesselSegment<2> >)) &VesselNetwork2::GetVesselSegmentIndex, 
            " " , py::arg("pVesselSegment") )
        .def(
            "GetVesselSegments", 
            (::std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >(VesselNetwork2::*)()) &VesselNetwork2::GetVesselSegments, 
            " "  )
        .def(
            "GetVesselSegment", 
            (::std::shared_ptr<VesselSegment<2> >(VesselNetwork2::*)(unsigned int)) &VesselNetwork2::GetVesselSegment, 
            " " , py::arg("index") )
        .def(
            "GetVessel", 
            (::std::shared_ptr<Vessel<2> >(VesselNetwork2::*)(unsigned int)) &VesselNetwork2::GetVessel, 
            " " , py::arg("index") )
        .def(
            "GetVessels", 
            (::std::vector<std::shared_ptr<Vessel<2> >, std::allocator<std::shared_ptr<Vessel<2> > > >(VesselNetwork2::*)()) &VesselNetwork2::GetVessels, 
            " "  )
        .def(
            "NodeIsInNetwork", 
            (bool(VesselNetwork2::*)(::std::shared_ptr<VesselNode<2> >)) &VesselNetwork2::NodeIsInNetwork, 
            " " , py::arg("pSourceNode") )
        .def(
            "MergeShortVessels", 
            (void(VesselNetwork2::*)(::QLength)) &VesselNetwork2::MergeShortVessels, 
            " " , py::arg("cutoff") = 10. * 1_um )
        .def(
            "MergeCoincidentNodes", 
            (void(VesselNetwork2::*)(double)) &VesselNetwork2::MergeCoincidentNodes, 
            " " , py::arg("tolerance") = 0. )
        .def(
            "MergeCoincidentNodes", 
            (void(VesselNetwork2::*)(::std::vector<std::shared_ptr<Vessel<2> >, std::allocator<std::shared_ptr<Vessel<2> > > >, double)) &VesselNetwork2::MergeCoincidentNodes, 
            " " , py::arg("pVessels"), py::arg("tolerance") = 0. )
        .def(
            "MergeCoincidentNodes", 
            (void(VesselNetwork2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >, double)) &VesselNetwork2::MergeCoincidentNodes, 
            " " , py::arg("nodes"), py::arg("tolerance") = 0. )
        .def(
            "Modified", 
            (void(VesselNetwork2::*)(bool, bool, bool)) &VesselNetwork2::Modified, 
            " " , py::arg("nodesOutOfDate") = true, py::arg("segmentsOutOfDate") = true, py::arg("vesselsOutOfDate") = true )
        .def(
            "RemoveVessel", 
            (void(VesselNetwork2::*)(::std::shared_ptr<Vessel<2> >, bool)) &VesselNetwork2::RemoveVessel, 
            " " , py::arg("pVessel"), py::arg("deleteVessel") = false )
        .def(
            "RemoveShortVessels", 
            (void(VesselNetwork2::*)(::QLength, bool)) &VesselNetwork2::RemoveShortVessels, 
            " " , py::arg("cutoff") = 10. * 1_um, py::arg("endsOnly") = true )
        .def(
            "Translate", 
            (void(VesselNetwork2::*)(::Vertex<2>)) &VesselNetwork2::Translate, 
            " " , py::arg("rTranslationVector") )
        .def(
            "Translate", 
            (void(VesselNetwork2::*)(::Vertex<2>, ::std::vector<std::shared_ptr<Vessel<2> >, std::allocator<std::shared_ptr<Vessel<2> > > >)) &VesselNetwork2::Translate, 
            " " , py::arg("rTranslationVector"), py::arg("vessels") )
        .def(
            "UpdateNodes", 
            (void(VesselNetwork2::*)()) &VesselNetwork2::UpdateNodes, 
            " "  )
        .def(
            "UpdateSegments", 
            (void(VesselNetwork2::*)()) &VesselNetwork2::UpdateSegments, 
            " "  )
        .def(
            "UpdateVesselNodes", 
            (void(VesselNetwork2::*)()) &VesselNetwork2::UpdateVesselNodes, 
            " "  )
        .def(
            "UpdateVesselIds", 
            (void(VesselNetwork2::*)()) &VesselNetwork2::UpdateVesselIds, 
            " "  )
        .def(
            "UpdateInternalVtkGeometry", 
            (void(VesselNetwork2::*)()) &VesselNetwork2::UpdateInternalVtkGeometry, 
            " "  )
        .def(
            "UpdateAll", 
            (void(VesselNetwork2::*)(bool)) &VesselNetwork2::UpdateAll, 
            " " , py::arg("merge") = false )
        .def(
            "Write", 
            (void(VesselNetwork2::*)(::std::string const &, bool)) &VesselNetwork2::Write, 
            " " , py::arg("rFileName"), py::arg("masterOnly") = true )
    ;
}
