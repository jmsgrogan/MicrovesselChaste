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
#include "ParabolicDiffusionReactionPde.hpp"

#include "PythonObjectConverters.hpp"
#include "ParabolicDiffusionReactionPde2_2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef ParabolicDiffusionReactionPde<2,2 > ParabolicDiffusionReactionPde2_2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class ParabolicDiffusionReactionPde2_2_Overloads : public ParabolicDiffusionReactionPde2_2{
    public:
    using ParabolicDiffusionReactionPde2_2::ParabolicDiffusionReactionPde;
    double ComputeSourceTerm(::ChastePoint<2> const & rX, double u, ::Element<2, 2> * pElement) override {
        PYBIND11_OVERLOAD(
            double,
            ParabolicDiffusionReactionPde2_2,
            ComputeSourceTerm,
            rX, 
u, 
pElement);
    }
    ::QConcentrationFlowRate ComputeSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            ParabolicDiffusionReactionPde2_2,
            ComputeSourceTerm,
            gridIndex, 
u);
    }
    ::QRate ComputeSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QRate,
            ParabolicDiffusionReactionPde2_2,
            ComputeSourceTermPrime,
            gridIndex, 
u);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            ParabolicDiffusionReactionPde2_2,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_ParabolicDiffusionReactionPde2_2_class(py::module &m){
py::class_<ParabolicDiffusionReactionPde2_2 , ParabolicDiffusionReactionPde2_2_Overloads , std::shared_ptr<ParabolicDiffusionReactionPde2_2 >  , AbstractDiscreteContinuumParabolicPde<2, 2>  >(m, "ParabolicDiffusionReactionPde2_2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ParabolicDiffusionReactionPde<2, 2> >(*)()) &ParabolicDiffusionReactionPde2_2::Create, 
            " "  )
        .def(
            "ComputeSourceTerm", 
            (double(ParabolicDiffusionReactionPde2_2::*)(::ChastePoint<2> const &, double, ::Element<2, 2> *)) &ParabolicDiffusionReactionPde2_2::ComputeSourceTerm, 
            " " , py::arg("rX"), py::arg("u"), py::arg("pElement") = __null )
        .def(
            "ComputeSourceTerm", 
            (::QConcentrationFlowRate(ParabolicDiffusionReactionPde2_2::*)(unsigned int, ::QConcentration)) &ParabolicDiffusionReactionPde2_2::ComputeSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeSourceTermPrime", 
            (::QRate(ParabolicDiffusionReactionPde2_2::*)(unsigned int, ::QConcentration)) &ParabolicDiffusionReactionPde2_2::ComputeSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "SetContinuumLinearInUTerm", 
            (void(ParabolicDiffusionReactionPde2_2::*)(::QRate)) &ParabolicDiffusionReactionPde2_2::SetContinuumLinearInUTerm, 
            " " , py::arg("linearInUTerm") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(ParabolicDiffusionReactionPde2_2::*)()) &ParabolicDiffusionReactionPde2_2::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
