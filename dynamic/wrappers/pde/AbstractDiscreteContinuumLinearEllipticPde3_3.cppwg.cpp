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

#include "AbstractDiscreteContinuumLinearEllipticPde3_3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumLinearEllipticPde<3,3 > AbstractDiscreteContinuumLinearEllipticPde3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::boost::numeric::ublas::c_matrix<double, 3, 3> _boost_numeric_ublas_c_matrix_lt_double_3_3_gt_;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;
typedef ::QRate _QRate;

class AbstractDiscreteContinuumLinearEllipticPde3_3_Overloads : public AbstractDiscreteContinuumLinearEllipticPde3_3{
    public:
    using AbstractDiscreteContinuumLinearEllipticPde3_3::AbstractDiscreteContinuumLinearEllipticPde;
    ::boost::numeric::ublas::c_matrix<double, 3, 3> ComputeDiffusionTerm(::ChastePoint<3> const & arg0) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_matrix_lt_double_3_3_gt_,
            AbstractDiscreteContinuumLinearEllipticPde3_3,
            ComputeDiffusionTerm,
            arg0);
    }
    ::QConcentrationFlowRate ComputeConstantInUSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD_PURE(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumLinearEllipticPde3_3,
            ComputeConstantInUSourceTerm,
            gridIndex);
    }
    ::QConcentrationFlowRate ComputeDiscreteConstantInUSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumLinearEllipticPde3_3,
            ComputeDiscreteConstantInUSourceTerm,
            gridIndex);
    }
    ::QRate ComputeLinearInUCoeffInSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD_PURE(
            _QRate,
            AbstractDiscreteContinuumLinearEllipticPde3_3,
            ComputeLinearInUCoeffInSourceTerm,
            gridIndex);
    }
    ::QRate ComputeDiscreteLinearInUCoeffInSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QRate,
            AbstractDiscreteContinuumLinearEllipticPde3_3,
            ComputeDiscreteLinearInUCoeffInSourceTerm,
            gridIndex);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumLinearEllipticPde3_3,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumLinearEllipticPde3_3_class(py::module &m){
py::class_<AbstractDiscreteContinuumLinearEllipticPde3_3 , AbstractDiscreteContinuumLinearEllipticPde3_3_Overloads , std::shared_ptr<AbstractDiscreteContinuumLinearEllipticPde3_3 >  , AbstractDiscreteContinuumPde<3, 3>  >(m, "AbstractDiscreteContinuumLinearEllipticPde3_3")
        .def(
            "ComputeDiffusionTerm", 
            (::boost::numeric::ublas::c_matrix<double, 3, 3>(AbstractDiscreteContinuumLinearEllipticPde3_3::*)(::ChastePoint<3> const &)) &AbstractDiscreteContinuumLinearEllipticPde3_3::ComputeDiffusionTerm, 
            " " , py::arg("arg0") )
        .def(
            "ComputeConstantInUSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumLinearEllipticPde3_3::*)(unsigned int)) &AbstractDiscreteContinuumLinearEllipticPde3_3::ComputeConstantInUSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeDiscreteConstantInUSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumLinearEllipticPde3_3::*)(unsigned int)) &AbstractDiscreteContinuumLinearEllipticPde3_3::ComputeDiscreteConstantInUSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeLinearInUCoeffInSourceTerm", 
            (::QRate(AbstractDiscreteContinuumLinearEllipticPde3_3::*)(unsigned int)) &AbstractDiscreteContinuumLinearEllipticPde3_3::ComputeLinearInUCoeffInSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeDiscreteLinearInUCoeffInSourceTerm", 
            (::QRate(AbstractDiscreteContinuumLinearEllipticPde3_3::*)(unsigned int)) &AbstractDiscreteContinuumLinearEllipticPde3_3::ComputeDiscreteLinearInUCoeffInSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "SetContinuumConstantInUTerm", 
            (void(AbstractDiscreteContinuumLinearEllipticPde3_3::*)(::QConcentrationFlowRate)) &AbstractDiscreteContinuumLinearEllipticPde3_3::SetContinuumConstantInUTerm, 
            " " , py::arg("constantInUTerm") )
        .def(
            "SetContinuumLinearInUTerm", 
            (void(AbstractDiscreteContinuumLinearEllipticPde3_3::*)(::QRate)) &AbstractDiscreteContinuumLinearEllipticPde3_3::SetContinuumLinearInUTerm, 
            " " , py::arg("linearInUTerm") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumLinearEllipticPde3_3::*)()) &AbstractDiscreteContinuumLinearEllipticPde3_3::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
