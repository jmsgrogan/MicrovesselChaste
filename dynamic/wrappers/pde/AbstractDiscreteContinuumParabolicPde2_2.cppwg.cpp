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
#include "AbstractDiscreteContinuumParabolicPde.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractDiscreteContinuumParabolicPde2_2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AbstractDiscreteContinuumParabolicPde<2,2 > AbstractDiscreteContinuumParabolicPde2_2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::boost::numeric::ublas::c_matrix<double, 2, 2> _boost_numeric_ublas_c_matrix_lt_double_2_2_gt_;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class AbstractDiscreteContinuumParabolicPde2_2_Overloads : public AbstractDiscreteContinuumParabolicPde2_2{
    public:
    using AbstractDiscreteContinuumParabolicPde2_2::AbstractDiscreteContinuumParabolicPde;
    ::boost::numeric::ublas::c_matrix<double, 2, 2> ComputeDiffusionTerm(::ChastePoint<2> const & rX, ::Element<2, 2> * pElement) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_matrix_lt_double_2_2_gt_,
            AbstractDiscreteContinuumParabolicPde2_2,
            ComputeDiffusionTerm,
            rX, 
pElement);
    }
    double ComputeDuDtCoefficientFunction(::ChastePoint<2> const & arg0) override {
        PYBIND11_OVERLOAD(
            double,
            AbstractDiscreteContinuumParabolicPde2_2,
            ComputeDuDtCoefficientFunction,
            arg0);
    }
    ::QConcentrationFlowRate ComputeSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD_PURE(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumParabolicPde2_2,
            ComputeSourceTerm,
            gridIndex, 
u);
    }
    ::QRate ComputeSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD_PURE(
            _QRate,
            AbstractDiscreteContinuumParabolicPde2_2,
            ComputeSourceTermPrime,
            gridIndex, 
u);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumParabolicPde2_2,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumParabolicPde2_2_class(py::module &m){
py::class_<AbstractDiscreteContinuumParabolicPde2_2 , AbstractDiscreteContinuumParabolicPde2_2_Overloads , std::shared_ptr<AbstractDiscreteContinuumParabolicPde2_2 >  , AbstractDiscreteContinuumPde<2, 2>  >(m, "AbstractDiscreteContinuumParabolicPde2_2")
        .def(
            "ComputeDiffusionTerm", 
            (::boost::numeric::ublas::c_matrix<double, 2, 2>(AbstractDiscreteContinuumParabolicPde2_2::*)(::ChastePoint<2> const &, ::Element<2, 2> *)) &AbstractDiscreteContinuumParabolicPde2_2::ComputeDiffusionTerm, 
            " " , py::arg("rX"), py::arg("pElement") = __null )
        .def(
            "ComputeDuDtCoefficientFunction", 
            (double(AbstractDiscreteContinuumParabolicPde2_2::*)(::ChastePoint<2> const &)) &AbstractDiscreteContinuumParabolicPde2_2::ComputeDuDtCoefficientFunction, 
            " " , py::arg("arg0") )
        .def(
            "ComputeSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumParabolicPde2_2::*)(unsigned int, ::QConcentration)) &AbstractDiscreteContinuumParabolicPde2_2::ComputeSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeSourceTermPrime", 
            (::QRate(AbstractDiscreteContinuumParabolicPde2_2::*)(unsigned int, ::QConcentration)) &AbstractDiscreteContinuumParabolicPde2_2::ComputeSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumParabolicPde2_2::*)()) &AbstractDiscreteContinuumParabolicPde2_2::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
