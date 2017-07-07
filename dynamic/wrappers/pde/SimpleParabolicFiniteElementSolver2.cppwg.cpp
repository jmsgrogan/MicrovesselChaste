#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "SimpleParabolicFiniteElementSolver.hpp"

#include "SimpleParabolicFiniteElementSolver2.cppwg.hpp"

namespace py = pybind11;
typedef SimpleParabolicFiniteElementSolver<2 > SimpleParabolicFiniteElementSolver2;
;

class SimpleParabolicFiniteElementSolver2_Overloads : public SimpleParabolicFiniteElementSolver2{
    public:
    using SimpleParabolicFiniteElementSolver2::SimpleParabolicFiniteElementSolver;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleParabolicFiniteElementSolver2,
            Solve,
            );
    }

};
void register_SimpleParabolicFiniteElementSolver2_class(py::module &m){
py::class_<SimpleParabolicFiniteElementSolver2 , SimpleParabolicFiniteElementSolver2_Overloads   >(m, "SimpleParabolicFiniteElementSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleParabolicFiniteElementSolver<2> >(*)()) &SimpleParabolicFiniteElementSolver2::Create, 
            " "  )
        .def(
            "rGetIntermediateSolutions", 
            (::std::vector<std::pair<std::vector<double, std::allocator<double> >, double>, std::allocator<std::pair<std::vector<double, std::allocator<double> >, double> > > const &(SimpleParabolicFiniteElementSolver2::*)()) &SimpleParabolicFiniteElementSolver2::rGetIntermediateSolutions, 
            " "  )
        .def(
            "SetTargetTimeIncrement", 
            (void(SimpleParabolicFiniteElementSolver2::*)(double)) &SimpleParabolicFiniteElementSolver2::SetTargetTimeIncrement, 
            " " , py::arg("targetIncrement") )
        .def(
            "SetStartTime", 
            (void(SimpleParabolicFiniteElementSolver2::*)(double)) &SimpleParabolicFiniteElementSolver2::SetStartTime, 
            " " , py::arg("startTime") )
        .def(
            "SetEndTime", 
            (void(SimpleParabolicFiniteElementSolver2::*)(double)) &SimpleParabolicFiniteElementSolver2::SetEndTime, 
            " " , py::arg("endTime") )
        .def(
            "SetInitialGuess", 
            (void(SimpleParabolicFiniteElementSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &SimpleParabolicFiniteElementSolver2::SetInitialGuess, 
            " " , py::arg("rInitialGuess") )
        .def(
            "SetStoreIntermediateSolutions", 
            (void(SimpleParabolicFiniteElementSolver2::*)(bool, unsigned int)) &SimpleParabolicFiniteElementSolver2::SetStoreIntermediateSolutions, 
            " " , py::arg("store"), py::arg("frequency") = 1 )
        .def(
            "SetWriteIntermediateSolutions", 
            (void(SimpleParabolicFiniteElementSolver2::*)(bool, unsigned int)) &SimpleParabolicFiniteElementSolver2::SetWriteIntermediateSolutions, 
            " " , py::arg("write"), py::arg("frequency") = 1 )
        .def(
            "Solve", 
            (void(SimpleParabolicFiniteElementSolver2::*)()) &SimpleParabolicFiniteElementSolver2::Solve, 
            " "  )
    ;
}
