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

#include "PythonObjectConverters.hpp"
#include "VesselNetwork3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef VesselNetwork<3 > VesselNetwork3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::shared_ptr<VesselNode<3> > _std_shared_ptr_lt_VesselNode_lt_3_gt__gt_;
typedef ::std::shared_ptr<Vessel<3> > _std_shared_ptr_lt_Vessel_lt_3_gt__gt_;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_;

class VesselNetwork3_Overloads : public VesselNetwork3{
    public:
    using VesselNetwork3::VesselNetwork;
    ::std::shared_ptr<VesselNode<3> > DivideVessel(::std::shared_ptr<Vessel<3> > pVessel, ::Vertex<3> const & rLocation) override {
        PYBIND11_OVERLOAD(
            _std_shared_ptr_lt_VesselNode_lt_3_gt__gt_,
            VesselNetwork3,
            DivideVessel,
            pVessel, 
rLocation);
    }
    void ExtendVessel(::std::shared_ptr<Vessel<3> > pVessel, ::std::shared_ptr<VesselNode<3> > pEndNode, ::std::shared_ptr<VesselNode<3> > pNewNode) override {
        PYBIND11_OVERLOAD(
            void,
            VesselNetwork3,
            ExtendVessel,
            pVessel, 
pEndNode, 
pNewNode);
    }
    ::std::shared_ptr<Vessel<3> > FormSprout(::std::shared_ptr<VesselNode<3> > pSproutBase, ::Vertex<3> const & sproutTipLocation) override {
        PYBIND11_OVERLOAD(
            _std_shared_ptr_lt_Vessel_lt_3_gt__gt_,
            VesselNetwork3,
            FormSprout,
            pSproutBase, 
sproutTipLocation);
    }
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD(
            _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_,
            VesselNetwork3,
            GetOutputData,
            );
    }

};
void register_VesselNetwork3_class(py::module &m){
py::class_<VesselNetwork3 , VesselNetwork3_Overloads , std::shared_ptr<VesselNetwork3 >  , AbstractVesselNetworkComponent<3>  >(m, "VesselNetwork3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetwork<3> >(*)()) &VesselNetwork3::Create, 
            " "  )
        .def(
            "AddVessel", 
            (void(VesselNetwork3::*)(::std::shared_ptr<Vessel<3> >)) &VesselNetwork3::AddVessel, 
            " " , py::arg("pVessel") )
        .def(
            "AddVessels", 
            (void(VesselNetwork3::*)(::std::vector<std::shared_ptr<Vessel<3> >, std::allocator<std::shared_ptr<Vessel<3> > > >)) &VesselNetwork3::AddVessels, 
            " " , py::arg("vessels") )
        .def(
            "ClearVessels", 
            (void(VesselNetwork3::*)()) &VesselNetwork3::ClearVessels, 
            " "  )
        .def(
            "CopyVessels", 
            (::std::vector<std::shared_ptr<Vessel<3> >, std::allocator<std::shared_ptr<Vessel<3> > > >(VesselNetwork3::*)()) &VesselNetwork3::CopyVessels, 
            " "  )
        .def(
            "CopyVessels", 
            (::std::vector<std::shared_ptr<Vessel<3> >, std::allocator<std::shared_ptr<Vessel<3> > > >(VesselNetwork3::*)(::std::vector<std::shared_ptr<Vessel<3> >, std::allocator<std::shared_ptr<Vessel<3> > > >)) &VesselNetwork3::CopyVessels, 
            " " , py::arg("vessels") )
        .def(
            "DivideVessel", 
            (::std::shared_ptr<VesselNode<3> >(VesselNetwork3::*)(::std::shared_ptr<Vessel<3> >, ::Vertex<3> const &)) &VesselNetwork3::DivideVessel, 
            " " , py::arg("pVessel"), py::arg("rLocation") )
        .def(
            "ExtendVessel", 
            (void(VesselNetwork3::*)(::std::shared_ptr<Vessel<3> >, ::std::shared_ptr<VesselNode<3> >, ::std::shared_ptr<VesselNode<3> >)) &VesselNetwork3::ExtendVessel, 
            " " , py::arg("pVessel"), py::arg("pEndNode"), py::arg("pNewNode") )
        .def(
            "FormSprout", 
            (::std::shared_ptr<Vessel<3> >(VesselNetwork3::*)(::std::shared_ptr<VesselNode<3> >, ::Vertex<3> const &)) &VesselNetwork3::FormSprout, 
            " " , py::arg("pSproutBase"), py::arg("sproutTipLocation") )
        .def(
            "GetNodeIndex", 
            (unsigned int(VesselNetwork3::*)(::std::shared_ptr<VesselNode<3> >)) &VesselNetwork3::GetNodeIndex, 
            " " , py::arg("pNode") )
        .def(
            "SetDistributedVectorFactory", 
            (void(VesselNetwork3::*)(::std::shared_ptr<DistributedVectorFactory>)) &VesselNetwork3::SetDistributedVectorFactory, 
            " " , py::arg("vectorFactory") )
        .def(
            "GetDistributedVectorFactory", 
            (::std::shared_ptr<DistributedVectorFactory>(VesselNetwork3::*)()) &VesselNetwork3::GetDistributedVectorFactory, 
            " "  )
        .def(
            "GetNode", 
            (::std::shared_ptr<VesselNode<3> >(VesselNetwork3::*)(unsigned int)) &VesselNetwork3::GetNode, 
            " " , py::arg("index") )
        .def(
            "GetNodes", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(VesselNetwork3::*)()) &VesselNetwork3::GetNodes, 
            " "  )
        .def(
            "GetVtk", 
            (::vtkSmartPointer<vtkPolyData>(VesselNetwork3::*)()) &VesselNetwork3::GetVtk, 
            " "  )
        .def(
            "GetVtkCellLocator", 
            (::vtkSmartPointer<vtkCellLocator>(VesselNetwork3::*)()) &VesselNetwork3::GetVtkCellLocator, 
            " "  )
        .def(
            "GetNumberOfNodes", 
            (unsigned int(VesselNetwork3::*)()) &VesselNetwork3::GetNumberOfNodes, 
            " "  )
        .def(
            "GetNumberOfNodesPerProcess", 
            (::std::vector<unsigned int, std::allocator<unsigned int> >(VesselNetwork3::*)()) &VesselNetwork3::GetNumberOfNodesPerProcess, 
            " "  )
        .def(
            "GetNumberOfVesselNodes", 
            (unsigned int(VesselNetwork3::*)()) &VesselNetwork3::GetNumberOfVesselNodes, 
            " "  )
        .def(
            "GetNumberOfVessels", 
            (unsigned int(VesselNetwork3::*)()) &VesselNetwork3::GetNumberOfVessels, 
            " "  )
        .def(
            "GetMaxBranchesOnNode", 
            (unsigned int(VesselNetwork3::*)()) &VesselNetwork3::GetMaxBranchesOnNode, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(VesselNetwork3::*)()) &VesselNetwork3::GetOutputData, 
            " "  )
        .def(
            "GetVesselEndNodes", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(VesselNetwork3::*)()) &VesselNetwork3::GetVesselEndNodes, 
            " "  )
        .def(
            "GetVesselIndex", 
            (unsigned int(VesselNetwork3::*)(::std::shared_ptr<Vessel<3> >)) &VesselNetwork3::GetVesselIndex, 
            " " , py::arg("pVessel") )
        .def(
            "GetVesselSegmentIndex", 
            (unsigned int(VesselNetwork3::*)(::std::shared_ptr<VesselSegment<3> >)) &VesselNetwork3::GetVesselSegmentIndex, 
            " " , py::arg("pVesselSegment") )
        .def(
            "GetVesselSegments", 
            (::std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >(VesselNetwork3::*)()) &VesselNetwork3::GetVesselSegments, 
            " "  )
        .def(
            "GetVesselSegment", 
            (::std::shared_ptr<VesselSegment<3> >(VesselNetwork3::*)(unsigned int)) &VesselNetwork3::GetVesselSegment, 
            " " , py::arg("index") )
        .def(
            "GetVessel", 
            (::std::shared_ptr<Vessel<3> >(VesselNetwork3::*)(unsigned int)) &VesselNetwork3::GetVessel, 
            " " , py::arg("index") )
        .def(
            "GetVessels", 
            (::std::vector<std::shared_ptr<Vessel<3> >, std::allocator<std::shared_ptr<Vessel<3> > > >(VesselNetwork3::*)()) &VesselNetwork3::GetVessels, 
            " "  )
        .def(
            "NodeIsInNetwork", 
            (bool(VesselNetwork3::*)(::std::shared_ptr<VesselNode<3> >)) &VesselNetwork3::NodeIsInNetwork, 
            " " , py::arg("pSourceNode") )
        .def(
            "MergeShortVessels", 
            (void(VesselNetwork3::*)(::QLength)) &VesselNetwork3::MergeShortVessels, 
            " " , py::arg("cutoff") = 10. * 1_um )
        .def(
            "MergeCoincidentNodes", 
            (void(VesselNetwork3::*)(double)) &VesselNetwork3::MergeCoincidentNodes, 
            " " , py::arg("tolerance") = 0. )
        .def(
            "MergeCoincidentNodes", 
            (void(VesselNetwork3::*)(::std::vector<std::shared_ptr<Vessel<3> >, std::allocator<std::shared_ptr<Vessel<3> > > >, double)) &VesselNetwork3::MergeCoincidentNodes, 
            " " , py::arg("pVessels"), py::arg("tolerance") = 0. )
        .def(
            "MergeCoincidentNodes", 
            (void(VesselNetwork3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >, double)) &VesselNetwork3::MergeCoincidentNodes, 
            " " , py::arg("nodes"), py::arg("tolerance") = 0. )
        .def(
            "Modified", 
            (void(VesselNetwork3::*)(bool, bool, bool)) &VesselNetwork3::Modified, 
            " " , py::arg("nodesOutOfDate") = true, py::arg("segmentsOutOfDate") = true, py::arg("vesselsOutOfDate") = true )
        .def(
            "RemoveVessel", 
            (void(VesselNetwork3::*)(::std::shared_ptr<Vessel<3> >, bool)) &VesselNetwork3::RemoveVessel, 
            " " , py::arg("pVessel"), py::arg("deleteVessel") = false )
        .def(
            "RemoveShortVessels", 
            (void(VesselNetwork3::*)(::QLength, bool)) &VesselNetwork3::RemoveShortVessels, 
            " " , py::arg("cutoff") = 10. * 1_um, py::arg("endsOnly") = true )
        .def(
            "Translate", 
            (void(VesselNetwork3::*)(::Vertex<3>)) &VesselNetwork3::Translate, 
            " " , py::arg("rTranslationVector") )
        .def(
            "Translate", 
            (void(VesselNetwork3::*)(::Vertex<3>, ::std::vector<std::shared_ptr<Vessel<3> >, std::allocator<std::shared_ptr<Vessel<3> > > >)) &VesselNetwork3::Translate, 
            " " , py::arg("rTranslationVector"), py::arg("vessels") )
        .def(
            "UpdateNodes", 
            (void(VesselNetwork3::*)()) &VesselNetwork3::UpdateNodes, 
            " "  )
        .def(
            "UpdateSegments", 
            (void(VesselNetwork3::*)()) &VesselNetwork3::UpdateSegments, 
            " "  )
        .def(
            "UpdateVesselNodes", 
            (void(VesselNetwork3::*)()) &VesselNetwork3::UpdateVesselNodes, 
            " "  )
        .def(
            "UpdateVesselIds", 
            (void(VesselNetwork3::*)()) &VesselNetwork3::UpdateVesselIds, 
            " "  )
        .def(
            "UpdateInternalVtkGeometry", 
            (void(VesselNetwork3::*)()) &VesselNetwork3::UpdateInternalVtkGeometry, 
            " "  )
        .def(
            "UpdateAll", 
            (void(VesselNetwork3::*)(bool)) &VesselNetwork3::UpdateAll, 
            " " , py::arg("merge") = false )
        .def(
            "Write", 
            (void(VesselNetwork3::*)(::std::string const &, bool)) &VesselNetwork3::Write, 
            " " , py::arg("rFileName"), py::arg("masterOnly") = true )
    ;
}
