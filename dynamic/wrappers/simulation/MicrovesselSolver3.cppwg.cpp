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
#include "MicrovesselSolver.hpp"

#include "MicrovesselSolver3.cppwg.hpp"

namespace py = pybind11;
typedef MicrovesselSolver<3 > MicrovesselSolver3;
;

void register_MicrovesselSolver3_class(py::module &m){
py::class_<MicrovesselSolver3    >(m, "MicrovesselSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MicrovesselSolver<3> >(*)()) &MicrovesselSolver3::Create, 
            " "  )
        .def(
            "AddDiscreteContinuumSolver", 
            (void(MicrovesselSolver3::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<3> >)) &MicrovesselSolver3::AddDiscreteContinuumSolver, 
            " " , py::arg("pDiscreteContinuumSolver") )
        .def(
            "AddMicrovesselModifier", 
            (void(MicrovesselSolver3::*)(::std::shared_ptr<AbstractMicrovesselModifier<3> >)) &MicrovesselSolver3::AddMicrovesselModifier, 
            " " , py::arg("pMicrovesselModifier") )
        .def(
            "GetDiscreteContinuumSolvers", 
            (::std::vector<std::shared_ptr<AbstractDiscreteContinuumSolver<3> >, std::allocator<std::shared_ptr<AbstractDiscreteContinuumSolver<3> > > >(MicrovesselSolver3::*)()) &MicrovesselSolver3::GetDiscreteContinuumSolvers, 
            " "  )
        .def(
            "Increment", 
            (void(MicrovesselSolver3::*)()) &MicrovesselSolver3::Increment, 
            " "  )
        .def(
            "Run", 
            (void(MicrovesselSolver3::*)()) &MicrovesselSolver3::Run, 
            " "  )
        .def(
            "SetAngiogenesisSolver", 
            (void(MicrovesselSolver3::*)(::std::shared_ptr<AngiogenesisSolver<3> >)) &MicrovesselSolver3::SetAngiogenesisSolver, 
            " " , py::arg("pAngiogenesisSolver") )
        .def(
            "SetOutputFileHandler", 
            (void(MicrovesselSolver3::*)(::std::shared_ptr<OutputFileHandler>)) &MicrovesselSolver3::SetOutputFileHandler, 
            " " , py::arg("pFileHandler") )
        .def(
            "SetOutputFrequency", 
            (void(MicrovesselSolver3::*)(unsigned int)) &MicrovesselSolver3::SetOutputFrequency, 
            " " , py::arg("frequency") )
        .def(
            "SetUpdatePdeEachSolve", 
            (void(MicrovesselSolver3::*)(bool)) &MicrovesselSolver3::SetUpdatePdeEachSolve, 
            " " , py::arg("doUpdate") )
        .def(
            "SetStructuralAdaptationSolver", 
            (void(MicrovesselSolver3::*)(::std::shared_ptr<StructuralAdaptationSolver<3> >)) &MicrovesselSolver3::SetStructuralAdaptationSolver, 
            " " , py::arg("pStructuralAdaptationSolver") )
        .def(
            "SetupFromModifier", 
            (void(MicrovesselSolver3::*)(::AbstractCellPopulation<3, 3> &, ::QLength, ::QConcentration, ::std::string const &)) &MicrovesselSolver3::SetupFromModifier, 
            " " , py::arg("rCellPopulation"), py::arg("cellReferenceLength"), py::arg("cellReferenceConcentration"), py::arg("rDirectory") )
        .def(
            "Setup", 
            (void(MicrovesselSolver3::*)()) &MicrovesselSolver3::Setup, 
            " "  )
        .def(
            "SetDiscreteContinuumSolversHaveCompatibleGridIndexing", 
            (void(MicrovesselSolver3::*)(bool)) &MicrovesselSolver3::SetDiscreteContinuumSolversHaveCompatibleGridIndexing, 
            " " , py::arg("compatibleIndexing") )
        .def(
            "SetVesselNetwork", 
            (void(MicrovesselSolver3::*)(::std::shared_ptr<VesselNetwork<3> >)) &MicrovesselSolver3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetCellPopulation", 
            (void(MicrovesselSolver3::*)(::std::shared_ptr<AbstractCellPopulation<3, 3> >)) &MicrovesselSolver3::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetRegressionSolver", 
            (void(MicrovesselSolver3::*)(::std::shared_ptr<RegressionSolver<3> >)) &MicrovesselSolver3::SetRegressionSolver, 
            " " , py::arg("pRegressionSolver") )
        .def(
            "UpdateCellData", 
            (void(MicrovesselSolver3::*)(::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >)) &MicrovesselSolver3::UpdateCellData, 
            " " , py::arg("labels") )
    ;
}
