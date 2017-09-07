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
#include "Owen2011OxygenBasedCellCycleModel.hpp"

#include "PythonObjectConverters.hpp"
#include "Owen2011OxygenBasedCellCycleModel.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef Owen2011OxygenBasedCellCycleModel Owen2011OxygenBasedCellCycleModel;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::AbstractCellCycleModel * _AbstractCellCycleModelPtr;

class Owen2011OxygenBasedCellCycleModel_Overloads : public Owen2011OxygenBasedCellCycleModel{
    public:
    using Owen2011OxygenBasedCellCycleModel::Owen2011OxygenBasedCellCycleModel;
    ::AbstractCellCycleModel * CreateCellCycleModel() override {
        PYBIND11_OVERLOAD(
            _AbstractCellCycleModelPtr,
            Owen2011OxygenBasedCellCycleModel,
            CreateCellCycleModel,
            );
    }
    double GetSDuration() const  override {
        PYBIND11_OVERLOAD(
            double,
            Owen2011OxygenBasedCellCycleModel,
            GetSDuration,
            );
    }
    double GetG2Duration() const  override {
        PYBIND11_OVERLOAD(
            double,
            Owen2011OxygenBasedCellCycleModel,
            GetG2Duration,
            );
    }
    double GetMDuration() const  override {
        PYBIND11_OVERLOAD(
            double,
            Owen2011OxygenBasedCellCycleModel,
            GetMDuration,
            );
    }
    void Initialise() override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011OxygenBasedCellCycleModel,
            Initialise,
            );
    }
    void InitialiseDaughterCell() override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011OxygenBasedCellCycleModel,
            InitialiseDaughterCell,
            );
    }
    void OutputCellCycleModelParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011OxygenBasedCellCycleModel,
            OutputCellCycleModelParameters,
            rParamsFile);
    }
    bool ReadyToDivide() override {
        PYBIND11_OVERLOAD(
            bool,
            Owen2011OxygenBasedCellCycleModel,
            ReadyToDivide,
            );
    }
    void ResetForDivision() override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011OxygenBasedCellCycleModel,
            ResetForDivision,
            );
    }
    void UpdateCellCyclePhase() override {
        PYBIND11_OVERLOAD(
            void,
            Owen2011OxygenBasedCellCycleModel,
            UpdateCellCyclePhase,
            );
    }

};
void register_Owen2011OxygenBasedCellCycleModel_class(py::module &m){
py::class_<Owen2011OxygenBasedCellCycleModel , Owen2011OxygenBasedCellCycleModel_Overloads , std::shared_ptr<Owen2011OxygenBasedCellCycleModel >   >(m, "Owen2011OxygenBasedCellCycleModel")
        .def(py::init<::boost::shared_ptr<AbstractCellCycleModelOdeSolver> >(), py::arg("pOdeSolver") = boost::shared_ptr<AbstractCellCycleModelOdeSolver>())
        .def(
            "CreateCellCycleModel", 
            (::AbstractCellCycleModel *(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::CreateCellCycleModel, 
            " "  , py::return_value_policy::reference)
        .def(
            "CheckAndLabelCell", 
            (void(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::CheckAndLabelCell, 
            " "  )
        .def(
            "GetCurrentQuiescentDuration", 
            (::QTime(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::GetCurrentQuiescentDuration, 
            " "  )
        .def(
            "GetCurrentQuiescenceOnsetTime", 
            (::QTime(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::GetCurrentQuiescenceOnsetTime, 
            " "  )
        .def(
            "GetEnterQuiescenceOxygenConcentration", 
            (::QPressure(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::GetEnterQuiescenceOxygenConcentration, 
            " "  )
        .def(
            "GetCriticalQuiescentDuration", 
            (::QTime(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::GetCriticalQuiescentDuration, 
            " "  )
        .def(
            "GetSDuration", 
            (double(Owen2011OxygenBasedCellCycleModel::*)() const ) &Owen2011OxygenBasedCellCycleModel::GetSDuration, 
            " "  )
        .def(
            "GetG2Duration", 
            (double(Owen2011OxygenBasedCellCycleModel::*)() const ) &Owen2011OxygenBasedCellCycleModel::GetG2Duration, 
            " "  )
        .def(
            "GetMDuration", 
            (double(Owen2011OxygenBasedCellCycleModel::*)() const ) &Owen2011OxygenBasedCellCycleModel::GetMDuration, 
            " "  )
        .def(
            "GetPhi", 
            (double(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::GetPhi, 
            " "  )
        .def(
            "GetVEGF", 
            (double(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::GetVEGF, 
            " "  )
        .def(
            "GetP53", 
            (double(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::GetP53, 
            " "  )
        .def(
            "GetLeaveQuiescenceOxygenConcentration", 
            (::QPressure(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::GetLeaveQuiescenceOxygenConcentration, 
            " "  )
        .def(
            "Initialise", 
            (void(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::Initialise, 
            " "  )
        .def(
            "InitialiseDaughterCell", 
            (void(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::InitialiseDaughterCell, 
            " "  )
        .def(
            "OutputCellCycleModelParameters", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::out_stream &)) &Owen2011OxygenBasedCellCycleModel::OutputCellCycleModelParameters, 
            " " , py::arg("rParamsFile") )
        .def(
            "ReadyToDivide", 
            (bool(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::ReadyToDivide, 
            " "  )
        .def(
            "ResetForDivision", 
            (void(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::ResetForDivision, 
            " "  )
        .def(
            "SetMaxRandInitialPhase", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QDimensionless)) &Owen2011OxygenBasedCellCycleModel::SetMaxRandInitialPhase, 
            " " , py::arg("rand_max_phase") )
        .def(
            "SetEnterQuiescenceOxygenConcentration", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QPressure)) &Owen2011OxygenBasedCellCycleModel::SetEnterQuiescenceOxygenConcentration, 
            " " , py::arg("enterQuiescenceOxygenConcentration") )
        .def(
            "SetLeaveQuiescenceOxygenConcentration", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QPressure)) &Owen2011OxygenBasedCellCycleModel::SetLeaveQuiescenceOxygenConcentration, 
            " " , py::arg("leaveQuiescenceOxygenConcentration") )
        .def(
            "SetCriticalQuiescentDuration", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QTime)) &Owen2011OxygenBasedCellCycleModel::SetCriticalQuiescentDuration, 
            " " , py::arg("criticalQuiescentDuration") )
        .def(
            "SetCurrentQuiescenceOnsetTime", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QTime)) &Owen2011OxygenBasedCellCycleModel::SetCurrentQuiescenceOnsetTime, 
            " " , py::arg("currentQuiescenceOnsetTime") )
        .def(
            "SetG2Onset", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QDimensionless)) &Owen2011OxygenBasedCellCycleModel::SetG2Onset, 
            " " , py::arg("value") )
        .def(
            "SetSOnset", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QDimensionless)) &Owen2011OxygenBasedCellCycleModel::SetSOnset, 
            " " , py::arg("value") )
        .def(
            "SetMOnset", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QDimensionless)) &Owen2011OxygenBasedCellCycleModel::SetMOnset, 
            " " , py::arg("value") )
        .def(
            "SetOdeSolverTimeStep", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QTime)) &Owen2011OxygenBasedCellCycleModel::SetOdeSolverTimeStep, 
            " " , py::arg("timeStep") )
        .def(
            "SetReferenceTimeScale", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QTime)) &Owen2011OxygenBasedCellCycleModel::SetReferenceTimeScale, 
            " " , py::arg("referenceTimeScale") )
        .def(
            "SetReferenceConcentrationScale", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(::QConcentration)) &Owen2011OxygenBasedCellCycleModel::SetReferenceConcentrationScale, 
            " " , py::arg("referenceConcentrationScale") )
        .def(
            "SetThresholdFractionOfNormalCellNeighbours", 
            (void(Owen2011OxygenBasedCellCycleModel::*)(double)) &Owen2011OxygenBasedCellCycleModel::SetThresholdFractionOfNormalCellNeighbours, 
            " " , py::arg("value") )
        .def(
            "UpdateQuiescentDuration", 
            (void(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::UpdateQuiescentDuration, 
            " "  )
        .def(
            "UpdateCellCyclePhase", 
            (void(Owen2011OxygenBasedCellCycleModel::*)()) &Owen2011OxygenBasedCellCycleModel::UpdateCellCyclePhase, 
            " "  )
    ;
}
