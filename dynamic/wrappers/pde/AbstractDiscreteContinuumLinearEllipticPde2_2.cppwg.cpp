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
#include "AbstractDiscreteContinuumLinearEllipticPde.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractDiscreteContinuumLinearEllipticPde2_2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AbstractDiscreteContinuumLinearEllipticPde<2,2 > AbstractDiscreteContinuumLinearEllipticPde2_2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::boost::numeric::ublas::c_matrix<double, 2, 2> _boost_numeric_ublas_c_matrix_lt_double_2_2_gt_;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;
typedef ::QRate _QRate;

class AbstractDiscreteContinuumLinearEllipticPde2_2_Overloads : public AbstractDiscreteContinuumLinearEllipticPde2_2{
    public:
    using AbstractDiscreteContinuumLinearEllipticPde2_2::AbstractDiscreteContinuumLinearEllipticPde;
    ::boost::numeric::ublas::c_matrix<double, 2, 2> ComputeDiffusionTerm(::ChastePoint<2> const & arg0) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_matrix_lt_double_2_2_gt_,
            AbstractDiscreteContinuumLinearEllipticPde2_2,
            ComputeDiffusionTerm,
            arg0);
    }
    ::QConcentrationFlowRate ComputeConstantInUSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD_PURE(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumLinearEllipticPde2_2,
            ComputeConstantInUSourceTerm,
            gridIndex);
    }
    ::QConcentrationFlowRate ComputeDiscreteConstantInUSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumLinearEllipticPde2_2,
            ComputeDiscreteConstantInUSourceTerm,
            gridIndex);
    }
    ::QRate ComputeLinearInUCoeffInSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD_PURE(
            _QRate,
            AbstractDiscreteContinuumLinearEllipticPde2_2,
            ComputeLinearInUCoeffInSourceTerm,
            gridIndex);
    }
    ::QRate ComputeDiscreteLinearInUCoeffInSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QRate,
            AbstractDiscreteContinuumLinearEllipticPde2_2,
            ComputeDiscreteLinearInUCoeffInSourceTerm,
            gridIndex);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumLinearEllipticPde2_2,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumLinearEllipticPde2_2_class(py::module &m){
py::class_<AbstractDiscreteContinuumLinearEllipticPde2_2 , AbstractDiscreteContinuumLinearEllipticPde2_2_Overloads , std::shared_ptr<AbstractDiscreteContinuumLinearEllipticPde2_2 >  , AbstractDiscreteContinuumPde<2, 2>  >(m, "AbstractDiscreteContinuumLinearEllipticPde2_2")
        .def(
            "ComputeDiffusionTerm", 
            (::boost::numeric::ublas::c_matrix<double, 2, 2>(AbstractDiscreteContinuumLinearEllipticPde2_2::*)(::ChastePoint<2> const &)) &AbstractDiscreteContinuumLinearEllipticPde2_2::ComputeDiffusionTerm, 
            " " , py::arg("arg0") )
        .def(
            "ComputeConstantInUSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumLinearEllipticPde2_2::*)(unsigned int)) &AbstractDiscreteContinuumLinearEllipticPde2_2::ComputeConstantInUSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeDiscreteConstantInUSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumLinearEllipticPde2_2::*)(unsigned int)) &AbstractDiscreteContinuumLinearEllipticPde2_2::ComputeDiscreteConstantInUSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeLinearInUCoeffInSourceTerm", 
            (::QRate(AbstractDiscreteContinuumLinearEllipticPde2_2::*)(unsigned int)) &AbstractDiscreteContinuumLinearEllipticPde2_2::ComputeLinearInUCoeffInSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeDiscreteLinearInUCoeffInSourceTerm", 
            (::QRate(AbstractDiscreteContinuumLinearEllipticPde2_2::*)(unsigned int)) &AbstractDiscreteContinuumLinearEllipticPde2_2::ComputeDiscreteLinearInUCoeffInSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "SetContinuumConstantInUTerm", 
            (void(AbstractDiscreteContinuumLinearEllipticPde2_2::*)(::QConcentrationFlowRate)) &AbstractDiscreteContinuumLinearEllipticPde2_2::SetContinuumConstantInUTerm, 
            " " , py::arg("constantInUTerm") )
        .def(
            "SetContinuumLinearInUTerm", 
            (void(AbstractDiscreteContinuumLinearEllipticPde2_2::*)(::QRate)) &AbstractDiscreteContinuumLinearEllipticPde2_2::SetContinuumLinearInUTerm, 
            " " , py::arg("linearInUTerm") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumLinearEllipticPde2_2::*)()) &AbstractDiscreteContinuumLinearEllipticPde2_2::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
