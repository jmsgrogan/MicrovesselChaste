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
#include "AbstractOdeSystem.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractOdeSystem.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AbstractOdeSystem AbstractOdeSystem;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractOdeSystem_Overloads : public AbstractOdeSystem{
    public:
    using AbstractOdeSystem::AbstractOdeSystem;
    void EvaluateYDerivatives(double time, ::std::vector<double, std::allocator<double> > const & rY, ::std::vector<double, std::allocator<double> > & rDY) override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractOdeSystem,
            EvaluateYDerivatives,
            time, 
rY, 
rDY);
    }
    bool CalculateStoppingEvent(double time, ::std::vector<double, std::allocator<double> > const & rY) override {
        PYBIND11_OVERLOAD(
            bool,
            AbstractOdeSystem,
            CalculateStoppingEvent,
            time, 
rY);
    }
    double CalculateRootFunction(double time, ::std::vector<double, std::allocator<double> > const & rY) override {
        PYBIND11_OVERLOAD(
            double,
            AbstractOdeSystem,
            CalculateRootFunction,
            time, 
rY);
    }

};
void register_AbstractOdeSystem_class(py::module &m){
py::class_<AbstractOdeSystem , AbstractOdeSystem_Overloads , std::shared_ptr<AbstractOdeSystem >   >(m, "AbstractOdeSystem")
        .def(
            "EvaluateYDerivatives", 
            (void(AbstractOdeSystem::*)(double, ::std::vector<double, std::allocator<double> > const &, ::std::vector<double, std::allocator<double> > &)) &AbstractOdeSystem::EvaluateYDerivatives, 
            " " , py::arg("time"), py::arg("rY"), py::arg("rDY") )
        .def(
            "CalculateStoppingEvent", 
            (bool(AbstractOdeSystem::*)(double, ::std::vector<double, std::allocator<double> > const &)) &AbstractOdeSystem::CalculateStoppingEvent, 
            " " , py::arg("time"), py::arg("rY") )
        .def(
            "CalculateRootFunction", 
            (double(AbstractOdeSystem::*)(double, ::std::vector<double, std::allocator<double> > const &)) &AbstractOdeSystem::CalculateRootFunction, 
            " " , py::arg("time"), py::arg("rY") )
        .def(
            "GetUseAnalyticJacobian", 
            (bool(AbstractOdeSystem::*)()) &AbstractOdeSystem::GetUseAnalyticJacobian, 
            " "  )
        .def(
            "rGetConstStateVariables", 
            (::std::vector<double, std::allocator<double> > const &(AbstractOdeSystem::*)() const ) &AbstractOdeSystem::rGetConstStateVariables, 
            " "  , py::return_value_policy::reference_internal)
    ;
}
