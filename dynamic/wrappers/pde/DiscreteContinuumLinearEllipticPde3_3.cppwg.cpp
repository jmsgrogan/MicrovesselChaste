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
#include "DiscreteContinuumLinearEllipticPde.hpp"

#include "DiscreteContinuumLinearEllipticPde3_3.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumLinearEllipticPde<3,3 > DiscreteContinuumLinearEllipticPde3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;
typedef ::QRate _QRate;

class DiscreteContinuumLinearEllipticPde3_3_Overloads : public DiscreteContinuumLinearEllipticPde3_3{
    public:
    using DiscreteContinuumLinearEllipticPde3_3::DiscreteContinuumLinearEllipticPde;
    double ComputeConstantInUSourceTerm(::ChastePoint<3> const & rX, ::Element<3, 3> * pElement) override {
        PYBIND11_OVERLOAD(
            double,
            DiscreteContinuumLinearEllipticPde3_3,
            ComputeConstantInUSourceTerm,
            rX, 
pElement);
    }
    ::QConcentrationFlowRate ComputeConstantInUSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            DiscreteContinuumLinearEllipticPde3_3,
            ComputeConstantInUSourceTerm,
            gridIndex);
    }
    ::QConcentrationFlowRate ComputeDiscreteConstantInUSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            DiscreteContinuumLinearEllipticPde3_3,
            ComputeDiscreteConstantInUSourceTerm,
            gridIndex);
    }
    double ComputeLinearInUCoeffInSourceTerm(::ChastePoint<3> const & rX, ::Element<3, 3> * pElement) override {
        PYBIND11_OVERLOAD(
            double,
            DiscreteContinuumLinearEllipticPde3_3,
            ComputeLinearInUCoeffInSourceTerm,
            rX, 
pElement);
    }
    ::QRate ComputeLinearInUCoeffInSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QRate,
            DiscreteContinuumLinearEllipticPde3_3,
            ComputeLinearInUCoeffInSourceTerm,
            gridIndex);
    }
    ::QRate ComputeDiscreteLinearInUCoeffInSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QRate,
            DiscreteContinuumLinearEllipticPde3_3,
            ComputeDiscreteLinearInUCoeffInSourceTerm,
            gridIndex);
    }

};
void register_DiscreteContinuumLinearEllipticPde3_3_class(py::module &m){
py::class_<DiscreteContinuumLinearEllipticPde3_3 , DiscreteContinuumLinearEllipticPde3_3_Overloads , std::shared_ptr<DiscreteContinuumLinearEllipticPde3_3 >  , AbstractDiscreteContinuumLinearEllipticPde<3, 3>  >(m, "DiscreteContinuumLinearEllipticPde3_3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumLinearEllipticPde<3, 3> >(*)()) &DiscreteContinuumLinearEllipticPde3_3::Create, 
            " "  )
        .def(
            "ComputeConstantInUSourceTerm", 
            (double(DiscreteContinuumLinearEllipticPde3_3::*)(::ChastePoint<3> const &, ::Element<3, 3> *)) &DiscreteContinuumLinearEllipticPde3_3::ComputeConstantInUSourceTerm, 
            " " , py::arg("rX"), py::arg("pElement") )
        .def(
            "ComputeConstantInUSourceTerm", 
            (::QConcentrationFlowRate(DiscreteContinuumLinearEllipticPde3_3::*)(unsigned int)) &DiscreteContinuumLinearEllipticPde3_3::ComputeConstantInUSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeDiscreteConstantInUSourceTerm", 
            (::QConcentrationFlowRate(DiscreteContinuumLinearEllipticPde3_3::*)(unsigned int)) &DiscreteContinuumLinearEllipticPde3_3::ComputeDiscreteConstantInUSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeLinearInUCoeffInSourceTerm", 
            (double(DiscreteContinuumLinearEllipticPde3_3::*)(::ChastePoint<3> const &, ::Element<3, 3> *)) &DiscreteContinuumLinearEllipticPde3_3::ComputeLinearInUCoeffInSourceTerm, 
            " " , py::arg("rX"), py::arg("pElement") )
        .def(
            "ComputeLinearInUCoeffInSourceTerm", 
            (::QRate(DiscreteContinuumLinearEllipticPde3_3::*)(unsigned int)) &DiscreteContinuumLinearEllipticPde3_3::ComputeLinearInUCoeffInSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeDiscreteLinearInUCoeffInSourceTerm", 
            (::QRate(DiscreteContinuumLinearEllipticPde3_3::*)(unsigned int)) &DiscreteContinuumLinearEllipticPde3_3::ComputeDiscreteLinearInUCoeffInSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
    ;
}
