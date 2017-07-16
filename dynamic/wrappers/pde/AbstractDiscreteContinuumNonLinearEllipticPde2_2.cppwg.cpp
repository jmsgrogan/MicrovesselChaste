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
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"

#include "AbstractDiscreteContinuumNonLinearEllipticPde2_2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumNonLinearEllipticPde<2,2 > AbstractDiscreteContinuumNonLinearEllipticPde2_2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::boost::numeric::ublas::c_matrix<double, 2, 2> _boost_numeric_ublas_c_matrix_lt_double_2_2_gt_;
typedef ::boost::numeric::ublas::c_matrix<double, 2, 2> _boost_numeric_ublas_c_matrix_lt_double_2_2_gt_;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class AbstractDiscreteContinuumNonLinearEllipticPde2_2_Overloads : public AbstractDiscreteContinuumNonLinearEllipticPde2_2{
    public:
    using AbstractDiscreteContinuumNonLinearEllipticPde2_2::AbstractDiscreteContinuumNonLinearEllipticPde;
    ::boost::numeric::ublas::c_matrix<double, 2, 2> ComputeDiffusionTerm(::ChastePoint<2> const & rX, double u) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_matrix_lt_double_2_2_gt_,
            AbstractDiscreteContinuumNonLinearEllipticPde2_2,
            ComputeDiffusionTerm,
            rX, 
u);
    }
    ::boost::numeric::ublas::c_matrix<double, 2, 2> ComputeDiffusionTermPrime(::ChastePoint<2> const & rX, double u) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_matrix_lt_double_2_2_gt_,
            AbstractDiscreteContinuumNonLinearEllipticPde2_2,
            ComputeDiffusionTermPrime,
            rX, 
u);
    }
    ::QConcentrationFlowRate ComputeLinearSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumNonLinearEllipticPde2_2,
            ComputeLinearSourceTerm,
            gridIndex);
    }
    ::QConcentrationFlowRate ComputeNonlinearSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD_PURE(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumNonLinearEllipticPde2_2,
            ComputeNonlinearSourceTerm,
            gridIndex, 
u);
    }
    ::QRate ComputeNonlinearSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD_PURE(
            _QRate,
            AbstractDiscreteContinuumNonLinearEllipticPde2_2,
            ComputeNonlinearSourceTermPrime,
            gridIndex, 
u);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumNonLinearEllipticPde2_2,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumNonLinearEllipticPde2_2_class(py::module &m){
py::class_<AbstractDiscreteContinuumNonLinearEllipticPde2_2 , AbstractDiscreteContinuumNonLinearEllipticPde2_2_Overloads , std::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde2_2 >  , AbstractDiscreteContinuumPde<2, 2>  >(m, "AbstractDiscreteContinuumNonLinearEllipticPde2_2")
        .def(
            "ComputeDiffusionTerm", 
            (::boost::numeric::ublas::c_matrix<double, 2, 2>(AbstractDiscreteContinuumNonLinearEllipticPde2_2::*)(::ChastePoint<2> const &, double)) &AbstractDiscreteContinuumNonLinearEllipticPde2_2::ComputeDiffusionTerm, 
            " " , py::arg("rX"), py::arg("u") )
        .def(
            "ComputeDiffusionTermPrime", 
            (::boost::numeric::ublas::c_matrix<double, 2, 2>(AbstractDiscreteContinuumNonLinearEllipticPde2_2::*)(::ChastePoint<2> const &, double)) &AbstractDiscreteContinuumNonLinearEllipticPde2_2::ComputeDiffusionTermPrime, 
            " " , py::arg("rX"), py::arg("u") )
        .def(
            "ComputeLinearSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumNonLinearEllipticPde2_2::*)(unsigned int)) &AbstractDiscreteContinuumNonLinearEllipticPde2_2::ComputeLinearSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeNonlinearSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumNonLinearEllipticPde2_2::*)(unsigned int, ::QConcentration)) &AbstractDiscreteContinuumNonLinearEllipticPde2_2::ComputeNonlinearSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeNonlinearSourceTermPrime", 
            (::QRate(AbstractDiscreteContinuumNonLinearEllipticPde2_2::*)(unsigned int, ::QConcentration)) &AbstractDiscreteContinuumNonLinearEllipticPde2_2::ComputeNonlinearSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumNonLinearEllipticPde2_2::*)()) &AbstractDiscreteContinuumNonLinearEllipticPde2_2::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
