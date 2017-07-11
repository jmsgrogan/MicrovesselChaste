#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "CornealMicropocketSimulation.hpp"

#include "CornealMicropocketSimulation2.cppwg.hpp"

namespace py = pybind11;
typedef CornealMicropocketSimulation<2 > CornealMicropocketSimulation2;
;

void register_CornealMicropocketSimulation2_class(py::module &m){
py::class_<CornealMicropocketSimulation2    >(m, "CornealMicropocketSimulation2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CornealMicropocketSimulation<2> >(*)()) &CornealMicropocketSimulation2::Create, 
            " "  )
        .def(
            "SetTipVelocity", 
            (void(CornealMicropocketSimulation2::*)(::QVelocity)) &CornealMicropocketSimulation2::SetTipVelocity, 
            " " , py::arg("velocity") )
        .def(
            "SetOnlyPerfusedSprout", 
            (void(CornealMicropocketSimulation2::*)(bool)) &CornealMicropocketSimulation2::SetOnlyPerfusedSprout, 
            " " , py::arg("onlyPerfused") )
        .def(
            "SetSampleFrequency", 
            (void(CornealMicropocketSimulation2::*)(unsigned int)) &CornealMicropocketSimulation2::SetSampleFrequency, 
            " " , py::arg("freq") )
        .def(
            "DoSampling", 
            (void(CornealMicropocketSimulation2::*)(::std::ofstream &, ::std::shared_ptr<AbstractDiscreteContinuumSolver<2> >, double, double, bool)) &CornealMicropocketSimulation2::DoSampling, 
            " " , py::arg("rStream"), py::arg("pSolver"), py::arg("time"), py::arg("multfact") = 1., py::arg("sampleOnce") = false )
        .def(
            "SetUpDomain", 
            (::std::shared_ptr<Part<2> >(CornealMicropocketSimulation2::*)()) &CornealMicropocketSimulation2::SetUpDomain, 
            " "  )
        .def(
            "SetUpGrid", 
            (::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >(CornealMicropocketSimulation2::*)(bool)) &CornealMicropocketSimulation2::SetUpGrid, 
            " " , py::arg("mSampling") = false )
        .def(
            "SetUpVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<2> >(CornealMicropocketSimulation2::*)()) &CornealMicropocketSimulation2::SetUpVesselNetwork, 
            " "  )
        .def(
            "SetUpSolver", 
            (void(CornealMicropocketSimulation2::*)()) &CornealMicropocketSimulation2::SetUpSolver, 
            " "  )
        .def(
            "SetWorkDir", 
            (void(CornealMicropocketSimulation2::*)(::std::string)) &CornealMicropocketSimulation2::SetWorkDir, 
            " " , py::arg("workDir") )
        .def(
            "SetUpSamplePoints", 
            (void(CornealMicropocketSimulation2::*)()) &CornealMicropocketSimulation2::SetUpSamplePoints, 
            " "  )
        .def(
            "SetDomainType", 
            (void(CornealMicropocketSimulation2::*)(::DomainType::Value)) &CornealMicropocketSimulation2::SetDomainType, 
            " " , py::arg("domainType") )
        .def(
            "SetCorneaRadius", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetCorneaRadius, 
            " " , py::arg("corneaRadius") )
        .def(
            "SetCorneaThickness", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetCorneaThickness, 
            " " , py::arg("corneaThickness") )
        .def(
            "SetPelletHeight", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetPelletHeight, 
            " " , py::arg("pelletHeight") )
        .def(
            "SetPelletThickness", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetPelletThickness, 
            " " , py::arg("pelletThickness") )
        .def(
            "SetPelletRadius", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetPelletRadius, 
            " " , py::arg("pelletRadius") )
        .def(
            "SetLimbalOffset", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetLimbalOffset, 
            " " , py::arg("limbalOffset") )
        .def(
            "SetAttractionStrength", 
            (void(CornealMicropocketSimulation2::*)(double)) &CornealMicropocketSimulation2::SetAttractionStrength, 
            " " , py::arg("attractionStrength") )
        .def(
            "SetChemotacticStrength", 
            (void(CornealMicropocketSimulation2::*)(double)) &CornealMicropocketSimulation2::SetChemotacticStrength, 
            " " , py::arg("chemotacticStrength") )
        .def(
            "SetDensityGridSpacing", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetDensityGridSpacing, 
            " " , py::arg("densityGridSpacing") )
        .def(
            "SetDoAnastamosis", 
            (void(CornealMicropocketSimulation2::*)(bool)) &CornealMicropocketSimulation2::SetDoAnastamosis, 
            " " , py::arg("doAnastamosis") )
        .def(
            "SetElementArea2d", 
            (void(CornealMicropocketSimulation2::*)(::QVolume)) &CornealMicropocketSimulation2::SetElementArea2d, 
            " " , py::arg("elementArea2d") )
        .def(
            "SetElementArea3d", 
            (void(CornealMicropocketSimulation2::*)(::QVolume)) &CornealMicropocketSimulation2::SetElementArea3d, 
            " " , py::arg("elementArea3d") )
        .def(
            "SetFinitePelletWidth", 
            (void(CornealMicropocketSimulation2::*)(bool)) &CornealMicropocketSimulation2::SetFinitePelletWidth, 
            " " , py::arg("finitePelletWidth") )
        .def(
            "SetGridSpacing", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetGridSpacing, 
            " " , py::arg("gridSpacing") )
        .def(
            "SetIncludeVesselSink", 
            (void(CornealMicropocketSimulation2::*)(bool)) &CornealMicropocketSimulation2::SetIncludeVesselSink, 
            " " , py::arg("includeVesselSink") )
        .def(
            "SetNodeSpacing", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetNodeSpacing, 
            " " , py::arg("nodeSpacing") )
        .def(
            "SetPdeTimeIncrement", 
            (void(CornealMicropocketSimulation2::*)(double)) &CornealMicropocketSimulation2::SetPdeTimeIncrement, 
            " " , py::arg("pdeTimeIncrement") )
        .def(
            "SetPelletConcentration", 
            (void(CornealMicropocketSimulation2::*)(::QConcentration)) &CornealMicropocketSimulation2::SetPelletConcentration, 
            " " , py::arg("pelletConcentration") )
        .def(
            "SetPersistenceAngle", 
            (void(CornealMicropocketSimulation2::*)(double)) &CornealMicropocketSimulation2::SetPersistenceAngle, 
            " " , py::arg("persistenceAngle") )
        .def(
            "SetRandomSeed", 
            (void(CornealMicropocketSimulation2::*)(unsigned int)) &CornealMicropocketSimulation2::SetRandomSeed, 
            " " , py::arg("randomSeed") )
        .def(
            "SetRunNumber", 
            (void(CornealMicropocketSimulation2::*)(unsigned int)) &CornealMicropocketSimulation2::SetRunNumber, 
            " " , py::arg("runNumber") )
        .def(
            "SetSampleSpacingX", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetSampleSpacingX, 
            " " , py::arg("sampleSpacingX") )
        .def(
            "SetSampleSpacingY", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetSampleSpacingY, 
            " " , py::arg("sampleSpacingY") )
        .def(
            "SetSampleSpacingZ", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetSampleSpacingZ, 
            " " , py::arg("sampleSpacingZ") )
        .def(
            "SetSproutingProbability", 
            (void(CornealMicropocketSimulation2::*)(::QRate)) &CornealMicropocketSimulation2::SetSproutingProbability, 
            " " , py::arg("sproutingProbability") )
        .def(
            "SetTimeStepSize", 
            (void(CornealMicropocketSimulation2::*)(::QTime)) &CornealMicropocketSimulation2::SetTimeStepSize, 
            " " , py::arg("timeStepSize") )
        .def(
            "SetTipExclusionRadius", 
            (void(CornealMicropocketSimulation2::*)(::QLength)) &CornealMicropocketSimulation2::SetTipExclusionRadius, 
            " " , py::arg("tipExclusionRadius") )
        .def(
            "SetTotalTime", 
            (void(CornealMicropocketSimulation2::*)(::QTime)) &CornealMicropocketSimulation2::SetTotalTime, 
            " " , py::arg("totalTime") )
        .def(
            "SetUptakeRatePerCell", 
            (void(CornealMicropocketSimulation2::*)(::QMolarFlowRate)) &CornealMicropocketSimulation2::SetUptakeRatePerCell, 
            " " , py::arg("uptakeRatePerCell") )
        .def(
            "SetUseFixedGradient", 
            (void(CornealMicropocketSimulation2::*)(bool)) &CornealMicropocketSimulation2::SetUseFixedGradient, 
            " " , py::arg("useFixedGradient") )
        .def(
            "SetUsePdeOnly", 
            (void(CornealMicropocketSimulation2::*)(bool)) &CornealMicropocketSimulation2::SetUsePdeOnly, 
            " " , py::arg("usePdeOnly") )
        .def(
            "SetUsePellet", 
            (void(CornealMicropocketSimulation2::*)(bool)) &CornealMicropocketSimulation2::SetUsePellet, 
            " " , py::arg("usePellet") )
        .def(
            "SetVegfBindingConstant", 
            (void(CornealMicropocketSimulation2::*)(double)) &CornealMicropocketSimulation2::SetVegfBindingConstant, 
            " " , py::arg("vegfBindingConstant") )
        .def(
            "SetVegfBloodConcentration", 
            (void(CornealMicropocketSimulation2::*)(::QConcentration)) &CornealMicropocketSimulation2::SetVegfBloodConcentration, 
            " " , py::arg("vegfBloodConcentration") )
        .def(
            "SetVegfDecayRate", 
            (void(CornealMicropocketSimulation2::*)(::QRate const)) &CornealMicropocketSimulation2::SetVegfDecayRate, 
            " " , py::arg("vegfDecayRate") )
        .def(
            "SetVegfDiffusivity", 
            (void(CornealMicropocketSimulation2::*)(::QDiffusivity)) &CornealMicropocketSimulation2::SetVegfDiffusivity, 
            " " , py::arg("vegfDiffusivity") )
        .def(
            "SetVegfPermeability", 
            (void(CornealMicropocketSimulation2::*)(::QMembranePermeability)) &CornealMicropocketSimulation2::SetVegfPermeability, 
            " " , py::arg("vegfPermeability") )
        .def(
            "Run", 
            (void(CornealMicropocketSimulation2::*)()) &CornealMicropocketSimulation2::Run, 
            " "  )
    ;
}