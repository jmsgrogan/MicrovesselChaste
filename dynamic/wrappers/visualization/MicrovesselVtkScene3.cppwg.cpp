#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "MicrovesselVtkScene.hpp"

#include "MicrovesselVtkScene3.cppwg.hpp"

namespace py = pybind11;
typedef MicrovesselVtkScene<3 > MicrovesselVtkScene3;
;

void register_MicrovesselVtkScene3_class(py::module &m){
py::class_<MicrovesselVtkScene3    >(m, "MicrovesselVtkScene3")
        .def(py::init< >())
        .def(
            "End", 
            (void(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::End, 
            " " )
        .def(
            "GetSceneAsCharBuffer", 
            (::vtkSmartPointer<vtkUnsignedCharArray>(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::GetSceneAsCharBuffer, 
            " " )
        .def(
            "GetRenderer", 
            (::vtkSmartPointer<vtkRenderer>(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::GetRenderer, 
            " " )
        .def(
            "GetPartActorGenerator", 
            (::std::shared_ptr<PartActorGenerator<3> >(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::GetPartActorGenerator, 
            " " )
        .def(
            "GetDiscreteContinuumMeshActorGenerator", 
            (::std::shared_ptr<DiscreteContinuumMeshActorGenerator<3> >(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::GetDiscreteContinuumMeshActorGenerator, 
            " " )
        .def(
            "GetRegularGridActorGenerator", 
            (::std::shared_ptr<RegularGridActorGenerator<3> >(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::GetRegularGridActorGenerator, 
            " " )
        .def(
            "GetVesselNetworkActorGenerator", 
            (::std::shared_ptr<VesselNetworkActorGenerator<3> >(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::GetVesselNetworkActorGenerator, 
            " " )
        .def(
            "GetCellPopulationActorGenerator", 
            (::std::shared_ptr<CellPopulationActorGenerator<3> >(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::GetCellPopulationActorGenerator, 
            " " )
        .def(
            "ResetRenderer", 
            (void(MicrovesselVtkScene3::*)(unsigned int)) &MicrovesselVtkScene3::ResetRenderer, 
            " " , py::arg("timeStep") = 0)
        .def(
            "Start", 
            (void(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::Start, 
            " " )
        .def(
            "SetCellPopulation", 
            (void(MicrovesselVtkScene3::*)(::std::shared_ptr<AbstractCellPopulation<3, 3> >)) &MicrovesselVtkScene3::SetCellPopulation, 
            " " , py::arg("pCellPopulation"))
        .def(
            "SetPart", 
            (void(MicrovesselVtkScene3::*)(::std::shared_ptr<Part<3> >)) &MicrovesselVtkScene3::SetPart, 
            " " , py::arg("pPart"))
        .def(
            "SetVesselNetwork", 
            (void(MicrovesselVtkScene3::*)(::std::shared_ptr<VesselNetwork<3> >)) &MicrovesselVtkScene3::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetRegularGrid", 
            (void(MicrovesselVtkScene3::*)(::std::shared_ptr<RegularGrid<3> >)) &MicrovesselVtkScene3::SetRegularGrid, 
            " " , py::arg("pGrid"))
        .def(
            "SetMesh", 
            (void(MicrovesselVtkScene3::*)(::std::shared_ptr<DiscreteContinuumMesh<3, 3> >)) &MicrovesselVtkScene3::SetMesh, 
            " " , py::arg("pMesh"))
        .def(
            "SetOutputFilePath", 
            (void(MicrovesselVtkScene3::*)(::std::string const &)) &MicrovesselVtkScene3::SetOutputFilePath, 
            " " , py::arg("rPath"))
        .def(
            "SetIsInteractive", 
            (void(MicrovesselVtkScene3::*)(bool)) &MicrovesselVtkScene3::SetIsInteractive, 
            " " , py::arg("isInteractive"))
        .def(
            "SetSaveAsAnimation", 
            (void(MicrovesselVtkScene3::*)(bool)) &MicrovesselVtkScene3::SetSaveAsAnimation, 
            " " , py::arg("saveAsAnimation"))
        .def(
            "SetSaveAsImages", 
            (void(MicrovesselVtkScene3::*)(bool)) &MicrovesselVtkScene3::SetSaveAsImages, 
            " " , py::arg("saveAsImages"))
        .def(
            "StartInteractiveEventHandler", 
            (void(MicrovesselVtkScene3::*)()) &MicrovesselVtkScene3::StartInteractiveEventHandler, 
            " " )
    ;
}
