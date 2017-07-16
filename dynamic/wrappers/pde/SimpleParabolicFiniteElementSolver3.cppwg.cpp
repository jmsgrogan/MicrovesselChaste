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
#include "SimpleParabolicFiniteElementSolver.hpp"

#include "SimpleParabolicFiniteElementSolver3.cppwg.hpp"

namespace py = pybind11;
typedef SimpleParabolicFiniteElementSolver<3 > SimpleParabolicFiniteElementSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class SimpleParabolicFiniteElementSolver3_Overloads : public SimpleParabolicFiniteElementSolver3{
    public:
    using SimpleParabolicFiniteElementSolver3::SimpleParabolicFiniteElementSolver;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleParabolicFiniteElementSolver3,
            Solve,
            );
    }

};
void register_SimpleParabolicFiniteElementSolver3_class(py::module &m){
py::class_<SimpleParabolicFiniteElementSolver3 , SimpleParabolicFiniteElementSolver3_Overloads , std::shared_ptr<SimpleParabolicFiniteElementSolver3 >  , AbstractFiniteElementSolverBase<3>  >(m, "SimpleParabolicFiniteElementSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleParabolicFiniteElementSolver<3> >(*)()) &SimpleParabolicFiniteElementSolver3::Create, 
            " "  )
        .def(
            "rGetIntermediateSolutions", 
            (::std::vector<std::pair<std::vector<double, std::allocator<double> >, double>, std::allocator<std::pair<std::vector<double, std::allocator<double> >, double> > > const &(SimpleParabolicFiniteElementSolver3::*)()) &SimpleParabolicFiniteElementSolver3::rGetIntermediateSolutions, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "SetTargetTimeIncrement", 
            (void(SimpleParabolicFiniteElementSolver3::*)(double)) &SimpleParabolicFiniteElementSolver3::SetTargetTimeIncrement, 
            " " , py::arg("targetIncrement") )
        .def(
            "SetStartTime", 
            (void(SimpleParabolicFiniteElementSolver3::*)(double)) &SimpleParabolicFiniteElementSolver3::SetStartTime, 
            " " , py::arg("startTime") )
        .def(
            "SetEndTime", 
            (void(SimpleParabolicFiniteElementSolver3::*)(double)) &SimpleParabolicFiniteElementSolver3::SetEndTime, 
            " " , py::arg("endTime") )
        .def(
            "SetInitialGuess", 
            (void(SimpleParabolicFiniteElementSolver3::*)(::std::vector<double, std::allocator<double> > const &)) &SimpleParabolicFiniteElementSolver3::SetInitialGuess, 
            " " , py::arg("rInitialGuess") )
        .def(
            "SetStoreIntermediateSolutions", 
            (void(SimpleParabolicFiniteElementSolver3::*)(bool, unsigned int)) &SimpleParabolicFiniteElementSolver3::SetStoreIntermediateSolutions, 
            " " , py::arg("store"), py::arg("frequency") = 1 )
        .def(
            "SetWriteIntermediateSolutions", 
            (void(SimpleParabolicFiniteElementSolver3::*)(bool, unsigned int)) &SimpleParabolicFiniteElementSolver3::SetWriteIntermediateSolutions, 
            " " , py::arg("write"), py::arg("frequency") = 1 )
        .def(
            "Solve", 
            (void(SimpleParabolicFiniteElementSolver3::*)()) &SimpleParabolicFiniteElementSolver3::Solve, 
            " "  )
    ;
}
