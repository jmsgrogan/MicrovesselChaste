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

#include "AbstractDiscreteContinuumParabolicPde3_3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumParabolicPde<3,3 > AbstractDiscreteContinuumParabolicPde3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::boost::numeric::ublas::c_matrix<double, 3, 3> _boost_numeric_ublas_c_matrix_lt_double_3_3_gt_;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class AbstractDiscreteContinuumParabolicPde3_3_Overloads : public AbstractDiscreteContinuumParabolicPde3_3{
    public:
    using AbstractDiscreteContinuumParabolicPde3_3::AbstractDiscreteContinuumParabolicPde;
    ::boost::numeric::ublas::c_matrix<double, 3, 3> ComputeDiffusionTerm(::ChastePoint<3> const & rX, ::Element<3, 3> * pElement) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_matrix_lt_double_3_3_gt_,
            AbstractDiscreteContinuumParabolicPde3_3,
            ComputeDiffusionTerm,
            rX, 
pElement);
    }
    double ComputeDuDtCoefficientFunction(::ChastePoint<3> const & arg0) override {
        PYBIND11_OVERLOAD(
            double,
            AbstractDiscreteContinuumParabolicPde3_3,
            ComputeDuDtCoefficientFunction,
            arg0);
    }
    ::QConcentrationFlowRate ComputeSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD_PURE(
            _QConcentrationFlowRate,
            AbstractDiscreteContinuumParabolicPde3_3,
            ComputeSourceTerm,
            gridIndex, 
u);
    }
    ::QRate ComputeSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD_PURE(
            _QRate,
            AbstractDiscreteContinuumParabolicPde3_3,
            ComputeSourceTermPrime,
            gridIndex, 
u);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumParabolicPde3_3,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumParabolicPde3_3_class(py::module &m){
py::class_<AbstractDiscreteContinuumParabolicPde3_3 , AbstractDiscreteContinuumParabolicPde3_3_Overloads , std::shared_ptr<AbstractDiscreteContinuumParabolicPde3_3 >  , AbstractDiscreteContinuumPde<3, 3>  >(m, "AbstractDiscreteContinuumParabolicPde3_3")
        .def(
            "ComputeDiffusionTerm", 
            (::boost::numeric::ublas::c_matrix<double, 3, 3>(AbstractDiscreteContinuumParabolicPde3_3::*)(::ChastePoint<3> const &, ::Element<3, 3> *)) &AbstractDiscreteContinuumParabolicPde3_3::ComputeDiffusionTerm, 
            " " , py::arg("rX"), py::arg("pElement") = __null )
        .def(
            "ComputeDuDtCoefficientFunction", 
            (double(AbstractDiscreteContinuumParabolicPde3_3::*)(::ChastePoint<3> const &)) &AbstractDiscreteContinuumParabolicPde3_3::ComputeDuDtCoefficientFunction, 
            " " , py::arg("arg0") )
        .def(
            "ComputeSourceTerm", 
            (::QConcentrationFlowRate(AbstractDiscreteContinuumParabolicPde3_3::*)(unsigned int, ::QConcentration)) &AbstractDiscreteContinuumParabolicPde3_3::ComputeSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeSourceTermPrime", 
            (::QRate(AbstractDiscreteContinuumParabolicPde3_3::*)(unsigned int, ::QConcentration)) &AbstractDiscreteContinuumParabolicPde3_3::ComputeSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumParabolicPde3_3::*)()) &AbstractDiscreteContinuumParabolicPde3_3::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
