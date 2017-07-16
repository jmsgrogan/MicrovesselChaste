#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "PythonObjectConverters.hpp"
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "vtkPolyData.h"
#include "MicrovesselVtkScene.hpp"

#include "MicrovesselVtkScene2.cppwg.hpp"

namespace py = pybind11;
typedef MicrovesselVtkScene<2 > MicrovesselVtkScene2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
PYBIND11_VTK_TYPECASTER(vtkRenderer);
PYBIND11_VTK_TYPECASTER(vtkUnsignedCharArray);

void register_MicrovesselVtkScene2_class(py::module &m){
py::class_<MicrovesselVtkScene2  , std::shared_ptr<MicrovesselVtkScene2 >   >(m, "MicrovesselVtkScene2")
        .def(py::init< >())
        .def(
            "End", 
            (void(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::End, 
            " "  )
        .def(
            "GetSceneAsCharBuffer", 
            (::vtkSmartPointer<vtkUnsignedCharArray>(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::GetSceneAsCharBuffer, 
            " "  )
        .def(
            "GetRenderer", 
            (::vtkSmartPointer<vtkRenderer>(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::GetRenderer, 
            " "  )
        .def(
            "GetPartActorGenerator", 
            (::std::shared_ptr<PartActorGenerator<2> >(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::GetPartActorGenerator, 
            " "  )
        .def(
            "GetDiscreteContinuumMeshActorGenerator", 
            (::std::shared_ptr<DiscreteContinuumMeshActorGenerator<2> >(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::GetDiscreteContinuumMeshActorGenerator, 
            " "  )
        .def(
            "GetRegularGridActorGenerator", 
            (::std::shared_ptr<RegularGridActorGenerator<2> >(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::GetRegularGridActorGenerator, 
            " "  )
        .def(
            "GetVesselNetworkActorGenerator", 
            (::std::shared_ptr<VesselNetworkActorGenerator<2> >(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::GetVesselNetworkActorGenerator, 
            " "  )
        .def(
            "GetCellPopulationActorGenerator", 
            (::std::shared_ptr<CellPopulationActorGenerator<2> >(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::GetCellPopulationActorGenerator, 
            " "  )
        .def(
            "ResetRenderer", 
            (void(MicrovesselVtkScene2::*)(unsigned int)) &MicrovesselVtkScene2::ResetRenderer, 
            " " , py::arg("timeStep") = 0 )
        .def(
            "Start", 
            (void(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::Start, 
            " "  )
        .def(
            "SetCellPopulation", 
            (void(MicrovesselVtkScene2::*)(::std::shared_ptr<AbstractCellPopulation<2, 2> >)) &MicrovesselVtkScene2::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetPart", 
            (void(MicrovesselVtkScene2::*)(::std::shared_ptr<Part<2> >)) &MicrovesselVtkScene2::SetPart, 
            " " , py::arg("pPart") )
        .def(
            "SetVesselNetwork", 
            (void(MicrovesselVtkScene2::*)(::std::shared_ptr<VesselNetwork<2> >)) &MicrovesselVtkScene2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetRegularGrid", 
            (void(MicrovesselVtkScene2::*)(::std::shared_ptr<RegularGrid<2> >)) &MicrovesselVtkScene2::SetRegularGrid, 
            " " , py::arg("pGrid") )
        .def(
            "SetMesh", 
            (void(MicrovesselVtkScene2::*)(::std::shared_ptr<DiscreteContinuumMesh<2, 2> >)) &MicrovesselVtkScene2::SetMesh, 
            " " , py::arg("pMesh") )
        .def(
            "SetOutputFilePath", 
            (void(MicrovesselVtkScene2::*)(::std::string const &)) &MicrovesselVtkScene2::SetOutputFilePath, 
            " " , py::arg("rPath") )
        .def(
            "SetIsInteractive", 
            (void(MicrovesselVtkScene2::*)(bool)) &MicrovesselVtkScene2::SetIsInteractive, 
            " " , py::arg("isInteractive") )
        .def(
            "SetSaveAsAnimation", 
            (void(MicrovesselVtkScene2::*)(bool)) &MicrovesselVtkScene2::SetSaveAsAnimation, 
            " " , py::arg("saveAsAnimation") )
        .def(
            "SetSaveAsImages", 
            (void(MicrovesselVtkScene2::*)(bool)) &MicrovesselVtkScene2::SetSaveAsImages, 
            " " , py::arg("saveAsImages") )
        .def(
            "StartInteractiveEventHandler", 
            (void(MicrovesselVtkScene2::*)()) &MicrovesselVtkScene2::StartInteractiveEventHandler, 
            " "  )
    ;
}
