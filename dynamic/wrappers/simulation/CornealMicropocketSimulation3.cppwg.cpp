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
#include "CornealMicropocketSimulation.hpp"

#include "PythonObjectConverters.hpp"
#include "CornealMicropocketSimulation3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef CornealMicropocketSimulation<3 > CornealMicropocketSimulation3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_CornealMicropocketSimulation3_class(py::module &m){
py::class_<CornealMicropocketSimulation3  , std::shared_ptr<CornealMicropocketSimulation3 >   >(m, "CornealMicropocketSimulation3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CornealMicropocketSimulation<3> >(*)()) &CornealMicropocketSimulation3::Create, 
            " "  )
        .def(
            "SetAnastamosisRadius", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetAnastamosisRadius, 
            " " , py::arg("radius") )
        .def(
            "GetAnastamosisRadius", 
            (::QLength(CornealMicropocketSimulation3::*)()) &CornealMicropocketSimulation3::GetAnastamosisRadius, 
            " "  )
        .def(
            "SetCellLength", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetCellLength, 
            " " , py::arg("length") )
        .def(
            "GetCellLength", 
            (::QLength(CornealMicropocketSimulation3::*)()) &CornealMicropocketSimulation3::GetCellLength, 
            " "  )
        .def(
            "SetTipVelocity", 
            (void(CornealMicropocketSimulation3::*)(::QVelocity)) &CornealMicropocketSimulation3::SetTipVelocity, 
            " " , py::arg("velocity") )
        .def(
            "SetOnlyPerfusedSprout", 
            (void(CornealMicropocketSimulation3::*)(bool)) &CornealMicropocketSimulation3::SetOnlyPerfusedSprout, 
            " " , py::arg("onlyPerfused") )
        .def(
            "SetSampleFrequency", 
            (void(CornealMicropocketSimulation3::*)(unsigned int)) &CornealMicropocketSimulation3::SetSampleFrequency, 
            " " , py::arg("freq") )
        .def(
            "DoSampling", 
            (void(CornealMicropocketSimulation3::*)(::std::ofstream &, ::vtkSmartPointer<vtkUnstructuredGrid>, ::std::string, double, double, bool)) &CornealMicropocketSimulation3::DoSampling, 
            " " , py::arg("rStream"), py::arg("pSampleGrid"), py::arg("sampleType"), py::arg("time"), py::arg("multfact") = 1., py::arg("sampleOnce") = false )
        .def(
            "SetUpDomain", 
            (::std::shared_ptr<Part<3> >(CornealMicropocketSimulation3::*)()) &CornealMicropocketSimulation3::SetUpDomain, 
            " "  )
        .def(
            "SetUpGrid", 
            (::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >(CornealMicropocketSimulation3::*)()) &CornealMicropocketSimulation3::SetUpGrid, 
            " "  )
        .def(
            "SetUpVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<3> >(CornealMicropocketSimulation3::*)()) &CornealMicropocketSimulation3::SetUpVesselNetwork, 
            " "  )
        .def(
            "SetUpSolver", 
            (void(CornealMicropocketSimulation3::*)()) &CornealMicropocketSimulation3::SetUpSolver, 
            " "  )
        .def(
            "SetWorkDir", 
            (void(CornealMicropocketSimulation3::*)(::std::string)) &CornealMicropocketSimulation3::SetWorkDir, 
            " " , py::arg("workDir") )
        .def(
            "SetUpSampleGrid", 
            (void(CornealMicropocketSimulation3::*)()) &CornealMicropocketSimulation3::SetUpSampleGrid, 
            " "  )
        .def(
            "SetDomainType", 
            (void(CornealMicropocketSimulation3::*)(::DomainType::Value)) &CornealMicropocketSimulation3::SetDomainType, 
            " " , py::arg("domainType") )
        .def(
            "SetCorneaRadius", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetCorneaRadius, 
            " " , py::arg("corneaRadius") )
        .def(
            "SetCorneaThickness", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetCorneaThickness, 
            " " , py::arg("corneaThickness") )
        .def(
            "SetPelletHeight", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetPelletHeight, 
            " " , py::arg("pelletHeight") )
        .def(
            "SetPelletThickness", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetPelletThickness, 
            " " , py::arg("pelletThickness") )
        .def(
            "SetPelletRadius", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetPelletRadius, 
            " " , py::arg("pelletRadius") )
        .def(
            "SetLimbalOffset", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetLimbalOffset, 
            " " , py::arg("limbalOffset") )
        .def(
            "SetAttractionStrength", 
            (void(CornealMicropocketSimulation3::*)(double)) &CornealMicropocketSimulation3::SetAttractionStrength, 
            " " , py::arg("attractionStrength") )
        .def(
            "SetChemotacticStrength", 
            (void(CornealMicropocketSimulation3::*)(double)) &CornealMicropocketSimulation3::SetChemotacticStrength, 
            " " , py::arg("chemotacticStrength") )
        .def(
            "SetDoAnastamosis", 
            (void(CornealMicropocketSimulation3::*)(bool)) &CornealMicropocketSimulation3::SetDoAnastamosis, 
            " " , py::arg("doAnastamosis") )
        .def(
            "SetElementArea2d", 
            (void(CornealMicropocketSimulation3::*)(::QVolume)) &CornealMicropocketSimulation3::SetElementArea2d, 
            " " , py::arg("elementArea2d") )
        .def(
            "SetElementArea3d", 
            (void(CornealMicropocketSimulation3::*)(::QVolume)) &CornealMicropocketSimulation3::SetElementArea3d, 
            " " , py::arg("elementArea3d") )
        .def(
            "SetFinitePelletWidth", 
            (void(CornealMicropocketSimulation3::*)(bool)) &CornealMicropocketSimulation3::SetFinitePelletWidth, 
            " " , py::arg("finitePelletWidth") )
        .def(
            "SetGridSpacing", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetGridSpacing, 
            " " , py::arg("gridSpacing") )
        .def(
            "SetIncludeVesselSink", 
            (void(CornealMicropocketSimulation3::*)(bool)) &CornealMicropocketSimulation3::SetIncludeVesselSink, 
            " " , py::arg("includeVesselSink") )
        .def(
            "SetNodeSpacing", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetNodeSpacing, 
            " " , py::arg("nodeSpacing") )
        .def(
            "SetPdeTimeIncrement", 
            (void(CornealMicropocketSimulation3::*)(double)) &CornealMicropocketSimulation3::SetPdeTimeIncrement, 
            " " , py::arg("pdeTimeIncrement") )
        .def(
            "SetPelletConcentration", 
            (void(CornealMicropocketSimulation3::*)(::QConcentration)) &CornealMicropocketSimulation3::SetPelletConcentration, 
            " " , py::arg("pelletConcentration") )
        .def(
            "SetPersistenceAngle", 
            (void(CornealMicropocketSimulation3::*)(double)) &CornealMicropocketSimulation3::SetPersistenceAngle, 
            " " , py::arg("persistenceAngle") )
        .def(
            "SetRandomSeed", 
            (void(CornealMicropocketSimulation3::*)(unsigned int)) &CornealMicropocketSimulation3::SetRandomSeed, 
            " " , py::arg("randomSeed") )
        .def(
            "SetRunNumber", 
            (void(CornealMicropocketSimulation3::*)(unsigned int)) &CornealMicropocketSimulation3::SetRunNumber, 
            " " , py::arg("runNumber") )
        .def(
            "SetSampleSpacingX", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetSampleSpacingX, 
            " " , py::arg("sampleSpacingX") )
        .def(
            "SetSampleSpacingY", 
            (void(CornealMicropocketSimulation3::*)(::QLength)) &CornealMicropocketSimulation3::SetSampleSpacingY, 
            " " , py::arg("sampleSpacingY") )
        .def(
            "SetSproutingProbability", 
            (void(CornealMicropocketSimulation3::*)(::QRate)) &CornealMicropocketSimulation3::SetSproutingProbability, 
            " " , py::arg("sproutingProbability") )
        .def(
            "SetTimeStepSize", 
            (void(CornealMicropocketSimulation3::*)(::QTime)) &CornealMicropocketSimulation3::SetTimeStepSize, 
            " " , py::arg("timeStepSize") )
        .def(
            "SetUseTipExclusion", 
            (void(CornealMicropocketSimulation3::*)(bool)) &CornealMicropocketSimulation3::SetUseTipExclusion, 
            " " , py::arg("usetipexclusion") )
        .def(
            "SetTotalTime", 
            (void(CornealMicropocketSimulation3::*)(::QTime)) &CornealMicropocketSimulation3::SetTotalTime, 
            " " , py::arg("totalTime") )
        .def(
            "SetUptakeRatePerCell", 
            (void(CornealMicropocketSimulation3::*)(::QMolarFlowRate)) &CornealMicropocketSimulation3::SetUptakeRatePerCell, 
            " " , py::arg("uptakeRatePerCell") )
        .def(
            "SetUseFixedGradient", 
            (void(CornealMicropocketSimulation3::*)(bool)) &CornealMicropocketSimulation3::SetUseFixedGradient, 
            " " , py::arg("useFixedGradient") )
        .def(
            "SetUsePdeOnly", 
            (void(CornealMicropocketSimulation3::*)(bool)) &CornealMicropocketSimulation3::SetUsePdeOnly, 
            " " , py::arg("usePdeOnly") )
        .def(
            "SetUsePellet", 
            (void(CornealMicropocketSimulation3::*)(bool)) &CornealMicropocketSimulation3::SetUsePellet, 
            " " , py::arg("usePellet") )
        .def(
            "SetVegfBindingConstant", 
            (void(CornealMicropocketSimulation3::*)(double)) &CornealMicropocketSimulation3::SetVegfBindingConstant, 
            " " , py::arg("vegfBindingConstant") )
        .def(
            "SetVegfBloodConcentration", 
            (void(CornealMicropocketSimulation3::*)(::QConcentration)) &CornealMicropocketSimulation3::SetVegfBloodConcentration, 
            " " , py::arg("vegfBloodConcentration") )
        .def(
            "SetVegfDecayRate", 
            (void(CornealMicropocketSimulation3::*)(::QRate const)) &CornealMicropocketSimulation3::SetVegfDecayRate, 
            " " , py::arg("vegfDecayRate") )
        .def(
            "SetVegfDiffusivity", 
            (void(CornealMicropocketSimulation3::*)(::QDiffusivity)) &CornealMicropocketSimulation3::SetVegfDiffusivity, 
            " " , py::arg("vegfDiffusivity") )
        .def(
            "SetVegfPermeability", 
            (void(CornealMicropocketSimulation3::*)(::QMembranePermeability)) &CornealMicropocketSimulation3::SetVegfPermeability, 
            " " , py::arg("vegfPermeability") )
        .def(
            "Run", 
            (void(CornealMicropocketSimulation3::*)()) &CornealMicropocketSimulation3::Run, 
            " "  )
    ;
}
