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
#include "ParabolicDiffusionReactionPde3_3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef ParabolicDiffusionReactionPde<3,3 > ParabolicDiffusionReactionPde3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class ParabolicDiffusionReactionPde3_3_Overloads : public ParabolicDiffusionReactionPde3_3{
    public:
    using ParabolicDiffusionReactionPde3_3::ParabolicDiffusionReactionPde;
    double ComputeSourceTerm(::ChastePoint<3> const & rX, double u, ::Element<3, 3> * pElement) override {
        PYBIND11_OVERLOAD(
            double,
            ParabolicDiffusionReactionPde3_3,
            ComputeSourceTerm,
            rX, 
u, 
pElement);
    }
    ::QConcentrationFlowRate ComputeSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            ParabolicDiffusionReactionPde3_3,
            ComputeSourceTerm,
            gridIndex, 
u);
    }
    ::QRate ComputeSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QRate,
            ParabolicDiffusionReactionPde3_3,
            ComputeSourceTermPrime,
            gridIndex, 
u);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            ParabolicDiffusionReactionPde3_3,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_ParabolicDiffusionReactionPde3_3_class(py::module &m){
py::class_<ParabolicDiffusionReactionPde3_3 , ParabolicDiffusionReactionPde3_3_Overloads , std::shared_ptr<ParabolicDiffusionReactionPde3_3 >  , AbstractDiscreteContinuumParabolicPde<3, 3>  >(m, "ParabolicDiffusionReactionPde3_3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ParabolicDiffusionReactionPde<3, 3> >(*)()) &ParabolicDiffusionReactionPde3_3::Create, 
            " "  )
        .def(
            "ComputeSourceTerm", 
            (double(ParabolicDiffusionReactionPde3_3::*)(::ChastePoint<3> const &, double, ::Element<3, 3> *)) &ParabolicDiffusionReactionPde3_3::ComputeSourceTerm, 
            " " , py::arg("rX"), py::arg("u"), py::arg("pElement") = __null )
        .def(
            "ComputeSourceTerm", 
            (::QConcentrationFlowRate(ParabolicDiffusionReactionPde3_3::*)(unsigned int, ::QConcentration)) &ParabolicDiffusionReactionPde3_3::ComputeSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeSourceTermPrime", 
            (::QRate(ParabolicDiffusionReactionPde3_3::*)(unsigned int, ::QConcentration)) &ParabolicDiffusionReactionPde3_3::ComputeSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "SetContinuumLinearInUTerm", 
            (void(ParabolicDiffusionReactionPde3_3::*)(::QRate)) &ParabolicDiffusionReactionPde3_3::SetContinuumLinearInUTerm, 
            " " , py::arg("linearInUTerm") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(ParabolicDiffusionReactionPde3_3::*)()) &ParabolicDiffusionReactionPde3_3::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
