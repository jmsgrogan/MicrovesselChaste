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

#include "AbstractDiscreteContinuumNonLinearEllipticPde3_3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumNonLinearEllipticPde<3,3 > AbstractDiscreteContinuumNonLinearEllipticPde3_3;
;
typedef ::boost::numeric::ublas::c_matrix<double, 3, 3> _boost_numeric_ublas_c_matrixdouble_3_3;
typedef ::boost::numeric::ublas::c_matrix<double, 3, 3> _boost_numeric_ublas_c_matrixdouble_3_3;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class AbstractDiscreteContinuumNonLinearEllipticPde3_3_Overloads : public AbstractDiscreteContinuumNonLinearEllipticPde3_3{
    public:
    using AbstractDiscreteContinuumNonLinearEllipticPde3_3::AbstractDiscreteContinuumNonLinearEllipticPde;
    ::boost::numeric::ublas::c_matrix<double, 3, 3> ComputeDiffusionTerm(::ChastePoint<3> const & rX, double u) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_matrixdouble_3_3,
            AbstractDiscreteContinuumNonLinearEllipticPde3_3,
            ComputeDiffusionTerm,
            rX, 
u);
    }
    ::boost::numeric::ublas::c_matrix<double, 3, 3> ComputeDiffusionTermPrime(::ChastePoint<3> const & rX, double u) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_matrixdouble_3_3,
            AbstractDiscreteContinuumNonLinearEllipticPde3_3,
            ComputeDiffusionTermPrime,
            rX, 
u);
    }
    ::QConcentrationFlowRate ComputeLinearSourceTerm(unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumNonLinearEllipticPde3_3,
            ComputeLinearSourceTerm,
            gridIndex);
    }
    ::QConcentrationFlowRate ComputeNonlinearSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD_PURE(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumNonLinearEllipticPde3_3,
            ComputeNonlinearSourceTerm,
            gridIndex, 
u);
    }
    ::QRate ComputeNonlinearSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD_PURE(
            _QRate,
            AbstractDiscreteContinuumNonLinearEllipticPde3_3,
            ComputeNonlinearSourceTermPrime,
            gridIndex, 
u);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumNonLinearEllipticPde3_3,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumNonLinearEllipticPde3_3_class(py::module &m){
py::class_<AbstractDiscreteContinuumNonLinearEllipticPde3_3 , AbstractDiscreteContinuumNonLinearEllipticPde3_3_Overloads   >(m, "AbstractDiscreteContinuumNonLinearEllipticPde3_3")
        .def(
            "ComputeDiffusionTerm", 
            (::boost::numeric::ublas::c_matrix<double, 3, 3>(AbstractDiscreteContinuumNonLinearEllipticPde3_3::*)(::ChastePoint<3> const &, double)) &AbstractDiscreteContinuumNonLinearEllipticPde3_3::ComputeDiffusionTerm, 
            " " , py::arg("rX"), py::arg("u") )
        .def(
            "ComputeDiffusionTermPrime", 
            (::boost::numeric::ublas::c_matrix<double, 3, 3>(AbstractDiscreteContinuumNonLinearEllipticPde3_3::*)(::ChastePoint<3> const &, double)) &AbstractDiscreteContinuumNonLinearEllipticPde3_3::ComputeDiffusionTermPrime, 
            " " , py::arg("rX"), py::arg("u") )
        .def(
            "ComputeLinearSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumNonLinearEllipticPde3_3::*)(unsigned int)) &AbstractDiscreteContinuumNonLinearEllipticPde3_3::ComputeLinearSourceTerm, 
            " " , py::arg("gridIndex") = 0 )
        .def(
            "ComputeNonlinearSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumNonLinearEllipticPde3_3::*)(unsigned int, ::QConcentration)) &AbstractDiscreteContinuumNonLinearEllipticPde3_3::ComputeNonlinearSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeNonlinearSourceTermPrime", 
            (::QRate(AbstractDiscreteContinuumNonLinearEllipticPde3_3::*)(unsigned int, ::QConcentration)) &AbstractDiscreteContinuumNonLinearEllipticPde3_3::ComputeNonlinearSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumNonLinearEllipticPde3_3::*)()) &AbstractDiscreteContinuumNonLinearEllipticPde3_3::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
