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
#include "MichaelisMentenSteadyStateDiffusionReactionPde.hpp"

#include "PythonObjectConverters.hpp"
#include "MichaelisMentenSteadyStateDiffusionReactionPde3_3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef MichaelisMentenSteadyStateDiffusionReactionPde<3,3 > MichaelisMentenSteadyStateDiffusionReactionPde3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class MichaelisMentenSteadyStateDiffusionReactionPde3_3_Overloads : public MichaelisMentenSteadyStateDiffusionReactionPde3_3{
    public:
    using MichaelisMentenSteadyStateDiffusionReactionPde3_3::MichaelisMentenSteadyStateDiffusionReactionPde;
    double ComputeLinearSourceTerm(::ChastePoint<3> const & rX) override {
        PYBIND11_OVERLOAD(
            double,
            MichaelisMentenSteadyStateDiffusionReactionPde3_3,
            ComputeLinearSourceTerm,
            rX);
    }
    double ComputeNonlinearSourceTerm(::ChastePoint<3> const & rX, double u) override {
        PYBIND11_OVERLOAD(
            double,
            MichaelisMentenSteadyStateDiffusionReactionPde3_3,
            ComputeNonlinearSourceTerm,
            rX, 
u);
    }
    ::QConcentrationFlowRate ComputeNonlinearSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            MichaelisMentenSteadyStateDiffusionReactionPde3_3,
            ComputeNonlinearSourceTerm,
            gridIndex, 
u);
    }
    double ComputeNonlinearSourceTermPrime(::ChastePoint<3> const & rX, double u) override {
        PYBIND11_OVERLOAD(
            double,
            MichaelisMentenSteadyStateDiffusionReactionPde3_3,
            ComputeNonlinearSourceTermPrime,
            rX, 
u);
    }
    ::QRate ComputeNonlinearSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QRate,
            MichaelisMentenSteadyStateDiffusionReactionPde3_3,
            ComputeNonlinearSourceTermPrime,
            gridIndex, 
u);
    }

};
void register_MichaelisMentenSteadyStateDiffusionReactionPde3_3_class(py::module &m){
py::class_<MichaelisMentenSteadyStateDiffusionReactionPde3_3 , MichaelisMentenSteadyStateDiffusionReactionPde3_3_Overloads , std::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde3_3 >  , AbstractDiscreteContinuumNonLinearEllipticPde<3, 3>  >(m, "MichaelisMentenSteadyStateDiffusionReactionPde3_3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<3, 3> >(*)()) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::Create, 
            " "  )
        .def(
            "ComputeLinearSourceTerm", 
            (double(MichaelisMentenSteadyStateDiffusionReactionPde3_3::*)(::ChastePoint<3> const &)) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::ComputeLinearSourceTerm, 
            " " , py::arg("rX") )
        .def(
            "ComputeNonlinearSourceTerm", 
            (double(MichaelisMentenSteadyStateDiffusionReactionPde3_3::*)(::ChastePoint<3> const &, double)) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::ComputeNonlinearSourceTerm, 
            " " , py::arg("rX"), py::arg("u") )
        .def(
            "ComputeNonlinearSourceTerm", 
            (::QConcentrationFlowRate(MichaelisMentenSteadyStateDiffusionReactionPde3_3::*)(unsigned int, ::QConcentration)) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::ComputeNonlinearSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeNonlinearSourceTermPrime", 
            (double(MichaelisMentenSteadyStateDiffusionReactionPde3_3::*)(::ChastePoint<3> const &, double)) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::ComputeNonlinearSourceTermPrime, 
            " " , py::arg("rX"), py::arg("u") )
        .def(
            "ComputeNonlinearSourceTermPrime", 
            (::QRate(MichaelisMentenSteadyStateDiffusionReactionPde3_3::*)(unsigned int, ::QConcentration)) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::ComputeNonlinearSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "SetRateConstant", 
            (void(MichaelisMentenSteadyStateDiffusionReactionPde3_3::*)(::QConcentrationFlowRate)) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::SetRateConstant, 
            " " , py::arg("rateConstant") )
        .def(
            "GetMichaelisMentenThreshold", 
            (::QConcentration(MichaelisMentenSteadyStateDiffusionReactionPde3_3::*)()) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::GetMichaelisMentenThreshold, 
            " "  )
        .def(
            "SetMichaelisMentenThreshold", 
            (void(MichaelisMentenSteadyStateDiffusionReactionPde3_3::*)(::QConcentration)) &MichaelisMentenSteadyStateDiffusionReactionPde3_3::SetMichaelisMentenThreshold, 
            " " , py::arg("threshold") )
    ;
}
