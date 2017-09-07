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
#include "MichaelisMentenSteadyStateDiffusionReactionPde2_2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef MichaelisMentenSteadyStateDiffusionReactionPde<2,2 > MichaelisMentenSteadyStateDiffusionReactionPde2_2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class MichaelisMentenSteadyStateDiffusionReactionPde2_2_Overloads : public MichaelisMentenSteadyStateDiffusionReactionPde2_2{
    public:
    using MichaelisMentenSteadyStateDiffusionReactionPde2_2::MichaelisMentenSteadyStateDiffusionReactionPde;
    double ComputeLinearSourceTerm(::ChastePoint<2> const & rX) override {
        PYBIND11_OVERLOAD(
            double,
            MichaelisMentenSteadyStateDiffusionReactionPde2_2,
            ComputeLinearSourceTerm,
            rX);
    }
    double ComputeNonlinearSourceTerm(::ChastePoint<2> const & rX, double u) override {
        PYBIND11_OVERLOAD(
            double,
            MichaelisMentenSteadyStateDiffusionReactionPde2_2,
            ComputeNonlinearSourceTerm,
            rX, 
u);
    }
    ::QConcentrationFlowRate ComputeNonlinearSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            MichaelisMentenSteadyStateDiffusionReactionPde2_2,
            ComputeNonlinearSourceTerm,
            gridIndex, 
u);
    }
    double ComputeNonlinearSourceTermPrime(::ChastePoint<2> const & rX, double u) override {
        PYBIND11_OVERLOAD(
            double,
            MichaelisMentenSteadyStateDiffusionReactionPde2_2,
            ComputeNonlinearSourceTermPrime,
            rX, 
u);
    }
    ::QRate ComputeNonlinearSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QRate,
            MichaelisMentenSteadyStateDiffusionReactionPde2_2,
            ComputeNonlinearSourceTermPrime,
            gridIndex, 
u);
    }

};
void register_MichaelisMentenSteadyStateDiffusionReactionPde2_2_class(py::module &m){
py::class_<MichaelisMentenSteadyStateDiffusionReactionPde2_2 , MichaelisMentenSteadyStateDiffusionReactionPde2_2_Overloads , std::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde2_2 >  , AbstractDiscreteContinuumNonLinearEllipticPde<2, 2>  >(m, "MichaelisMentenSteadyStateDiffusionReactionPde2_2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MichaelisMentenSteadyStateDiffusionReactionPde<2, 2> >(*)()) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::Create, 
            " "  )
        .def(
            "ComputeLinearSourceTerm", 
            (double(MichaelisMentenSteadyStateDiffusionReactionPde2_2::*)(::ChastePoint<2> const &)) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::ComputeLinearSourceTerm, 
            " " , py::arg("rX") )
        .def(
            "ComputeNonlinearSourceTerm", 
            (double(MichaelisMentenSteadyStateDiffusionReactionPde2_2::*)(::ChastePoint<2> const &, double)) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::ComputeNonlinearSourceTerm, 
            " " , py::arg("rX"), py::arg("u") )
        .def(
            "ComputeNonlinearSourceTerm", 
            (::QConcentrationFlowRate(MichaelisMentenSteadyStateDiffusionReactionPde2_2::*)(unsigned int, ::QConcentration)) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::ComputeNonlinearSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeNonlinearSourceTermPrime", 
            (double(MichaelisMentenSteadyStateDiffusionReactionPde2_2::*)(::ChastePoint<2> const &, double)) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::ComputeNonlinearSourceTermPrime, 
            " " , py::arg("rX"), py::arg("u") )
        .def(
            "ComputeNonlinearSourceTermPrime", 
            (::QRate(MichaelisMentenSteadyStateDiffusionReactionPde2_2::*)(unsigned int, ::QConcentration)) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::ComputeNonlinearSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "SetRateConstant", 
            (void(MichaelisMentenSteadyStateDiffusionReactionPde2_2::*)(::QConcentrationFlowRate)) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::SetRateConstant, 
            " " , py::arg("rateConstant") )
        .def(
            "GetMichaelisMentenThreshold", 
            (::QConcentration(MichaelisMentenSteadyStateDiffusionReactionPde2_2::*)()) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::GetMichaelisMentenThreshold, 
            " "  )
        .def(
            "SetMichaelisMentenThreshold", 
            (void(MichaelisMentenSteadyStateDiffusionReactionPde2_2::*)(::QConcentration)) &MichaelisMentenSteadyStateDiffusionReactionPde2_2::SetMichaelisMentenThreshold, 
            " " , py::arg("threshold") )
    ;
}
