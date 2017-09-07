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

#include "PythonObjectConverters.hpp"
#include "MicrovesselSolver2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef MicrovesselSolver<2 > MicrovesselSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_MicrovesselSolver2_class(py::module &m){
py::class_<MicrovesselSolver2  , std::shared_ptr<MicrovesselSolver2 >   >(m, "MicrovesselSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MicrovesselSolver<2> >(*)()) &MicrovesselSolver2::Create, 
            " "  )
        .def(
            "AddDiscreteContinuumSolver", 
            (void(MicrovesselSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<2> >)) &MicrovesselSolver2::AddDiscreteContinuumSolver, 
            " " , py::arg("pDiscreteContinuumSolver") )
        .def(
            "AddMicrovesselModifier", 
            (void(MicrovesselSolver2::*)(::std::shared_ptr<AbstractMicrovesselModifier<2> >)) &MicrovesselSolver2::AddMicrovesselModifier, 
            " " , py::arg("pMicrovesselModifier") )
        .def(
            "GetDiscreteContinuumSolvers", 
            (::std::vector<std::shared_ptr<AbstractDiscreteContinuumSolver<2> >, std::allocator<std::shared_ptr<AbstractDiscreteContinuumSolver<2> > > >(MicrovesselSolver2::*)()) &MicrovesselSolver2::GetDiscreteContinuumSolvers, 
            " "  )
        .def(
            "Increment", 
            (void(MicrovesselSolver2::*)()) &MicrovesselSolver2::Increment, 
            " "  )
        .def(
            "Run", 
            (void(MicrovesselSolver2::*)()) &MicrovesselSolver2::Run, 
            " "  )
        .def(
            "SetAngiogenesisSolver", 
            (void(MicrovesselSolver2::*)(::std::shared_ptr<AngiogenesisSolver<2> >)) &MicrovesselSolver2::SetAngiogenesisSolver, 
            " " , py::arg("pAngiogenesisSolver") )
        .def(
            "SetOutputFileHandler", 
            (void(MicrovesselSolver2::*)(::std::shared_ptr<OutputFileHandler>)) &MicrovesselSolver2::SetOutputFileHandler, 
            " " , py::arg("pFileHandler") )
        .def(
            "SetOutputFrequency", 
            (void(MicrovesselSolver2::*)(unsigned int)) &MicrovesselSolver2::SetOutputFrequency, 
            " " , py::arg("frequency") )
        .def(
            "SetUpdatePdeEachSolve", 
            (void(MicrovesselSolver2::*)(bool)) &MicrovesselSolver2::SetUpdatePdeEachSolve, 
            " " , py::arg("doUpdate") )
        .def(
            "SetStructuralAdaptationSolver", 
            (void(MicrovesselSolver2::*)(::std::shared_ptr<StructuralAdaptationSolver<2> >)) &MicrovesselSolver2::SetStructuralAdaptationSolver, 
            " " , py::arg("pStructuralAdaptationSolver") )
        .def(
            "SetupFromModifier", 
            (void(MicrovesselSolver2::*)(::AbstractCellPopulation<2, 2> &, ::QLength, ::QConcentration, ::std::string const &)) &MicrovesselSolver2::SetupFromModifier, 
            " " , py::arg("rCellPopulation"), py::arg("cellReferenceLength"), py::arg("cellReferenceConcentration"), py::arg("rDirectory") )
        .def(
            "Setup", 
            (void(MicrovesselSolver2::*)()) &MicrovesselSolver2::Setup, 
            " "  )
        .def(
            "SetDiscreteContinuumSolversHaveCompatibleGridIndexing", 
            (void(MicrovesselSolver2::*)(bool)) &MicrovesselSolver2::SetDiscreteContinuumSolversHaveCompatibleGridIndexing, 
            " " , py::arg("compatibleIndexing") )
        .def(
            "SetVesselNetwork", 
            (void(MicrovesselSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &MicrovesselSolver2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetCellPopulation", 
            (void(MicrovesselSolver2::*)(::std::shared_ptr<AbstractCellPopulation<2, 2> >)) &MicrovesselSolver2::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetRegressionSolver", 
            (void(MicrovesselSolver2::*)(::std::shared_ptr<RegressionSolver<2> >)) &MicrovesselSolver2::SetRegressionSolver, 
            " " , py::arg("pRegressionSolver") )
        .def(
            "UpdateCellData", 
            (void(MicrovesselSolver2::*)(::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >)) &MicrovesselSolver2::UpdateCellData, 
            " " , py::arg("labels") )
    ;
}
