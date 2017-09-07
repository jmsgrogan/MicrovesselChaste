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

#include "PythonObjectConverters.hpp"
#include "DiscreteContinuumLinearEllipticPde2_2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef DiscreteContinuumLinearEllipticPde<2,2 > DiscreteContinuumLinearEllipticPde2_2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;
typedef ::QRate _QRate;

class DiscreteContinuumLinearEllipticPde2_2_Overloads : public DiscreteContinuumLinearEllipticPde2_2{
    public:
    using DiscreteContinuumLinearEllipticPde2_2::DiscreteContinuumLinearEllipticPde;
    double ComputeConstantInUSourceTerm(::ChastePoint<2> const & rX, ::Element<2, 2> * pElement) override {
        PYBIND11_OVERLOAD(
            double,
            DiscreteContinuumLinearEllipticPde2_2,
            ComputeConstantInUSourceTerm,
            rX, 
pElement);
    }
    ::QConcentrationFlowRate ComputeConstantInUSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            DiscreteContinuumLinearEllipticPde2_2,
            ComputeConstantInUSourceTerm,
            gridIndex);
    }
    ::QConcentrationFlowRate ComputeDiscreteConstantInUSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            DiscreteContinuumLinearEllipticPde2_2,
            ComputeDiscreteConstantInUSourceTerm,
            gridIndex);
    }
    double ComputeLinearInUCoeffInSourceTerm(::ChastePoint<2> const & rX, ::Element<2, 2> * pElement) override {
        PYBIND11_OVERLOAD(
            double,
            DiscreteContinuumLinearEllipticPde2_2,
            ComputeLinearInUCoeffInSourceTerm,
            rX, 
pElement);
    }
    ::QRate ComputeLinearInUCoeffInSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QRate,
            DiscreteContinuumLinearEllipticPde2_2,
            ComputeLinearInUCoeffInSourceTerm,
            gridIndex);
    }
    ::QRate ComputeDiscreteLinearInUCoeffInSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QRate,
            DiscreteContinuumLinearEllipticPde2_2,
            ComputeDiscreteLinearInUCoeffInSourceTerm,
            gridIndex);
    }

};
void register_DiscreteContinuumLinearEllipticPde2_2_class(py::module &m){
py::class_<DiscreteContinuumLinearEllipticPde2_2 , DiscreteContinuumLinearEllipticPde2_2_Overloads , std::shared_ptr<DiscreteContinuumLinearEllipticPde2_2 >  , AbstractDiscreteContinuumLinearEllipticPde<2, 2>  >(m, "DiscreteContinuumLinearEllipticPde2_2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumLinearEllipticPde<2, 2> >(*)()) &DiscreteContinuumLinearEllipticPde2_2::Create, 
            " "  )
        .def(
            "ComputeConstantInUSourceTerm", 
            (double(DiscreteContinuumLinearEllipticPde2_2::*)(::ChastePoint<2> const &, ::Element<2, 2> *)) &DiscreteContinuumLinearEllipticPde2_2::ComputeConstantInUSourceTerm, 
            " " , py::arg("rX"), py::arg("pElement") )
        .def(
            "ComputeConstantInUSourceTerm", 
            (::QConcentrationFlowRate(DiscreteContinuumLinearEllipticPde2_2::*)(unsigned int)) &DiscreteContinuumLinearEllipticPde2_2::ComputeConstantInUSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeDiscreteConstantInUSourceTerm", 
            (::QConcentrationFlowRate(DiscreteContinuumLinearEllipticPde2_2::*)(unsigned int)) &DiscreteContinuumLinearEllipticPde2_2::ComputeDiscreteConstantInUSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeLinearInUCoeffInSourceTerm", 
            (double(DiscreteContinuumLinearEllipticPde2_2::*)(::ChastePoint<2> const &, ::Element<2, 2> *)) &DiscreteContinuumLinearEllipticPde2_2::ComputeLinearInUCoeffInSourceTerm, 
            " " , py::arg("rX"), py::arg("pElement") )
        .def(
            "ComputeLinearInUCoeffInSourceTerm", 
            (::QRate(DiscreteContinuumLinearEllipticPde2_2::*)(unsigned int)) &DiscreteContinuumLinearEllipticPde2_2::ComputeLinearInUCoeffInSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeDiscreteLinearInUCoeffInSourceTerm", 
            (::QRate(DiscreteContinuumLinearEllipticPde2_2::*)(unsigned int)) &DiscreteContinuumLinearEllipticPde2_2::ComputeDiscreteLinearInUCoeffInSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
    ;
}
