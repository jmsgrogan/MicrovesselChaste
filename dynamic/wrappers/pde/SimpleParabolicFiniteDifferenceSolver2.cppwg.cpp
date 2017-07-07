#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "SimpleParabolicFiniteDifferenceSolver.hpp"

#include "SimpleParabolicFiniteDifferenceSolver2.cppwg.hpp"

namespace py = pybind11;
typedef SimpleParabolicFiniteDifferenceSolver<2 > SimpleParabolicFiniteDifferenceSolver2;
;

class SimpleParabolicFiniteDifferenceSolver2_Overloads : public SimpleParabolicFiniteDifferenceSolver2{
    public:
    using SimpleParabolicFiniteDifferenceSolver2::SimpleParabolicFiniteDifferenceSolver;
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleParabolicFiniteDifferenceSolver2,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleParabolicFiniteDifferenceSolver2,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleParabolicFiniteDifferenceSolver2,
            Solve,
            );
    }

};
void register_SimpleParabolicFiniteDifferenceSolver2_class(py::module &m){
py::class_<SimpleParabolicFiniteDifferenceSolver2 , SimpleParabolicFiniteDifferenceSolver2_Overloads   >(m, "SimpleParabolicFiniteDifferenceSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleParabolicFiniteDifferenceSolver<2> >(*)()) &SimpleParabolicFiniteDifferenceSolver2::Create, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(SimpleParabolicFiniteDifferenceSolver2::*)()) &SimpleParabolicFiniteDifferenceSolver2::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(SimpleParabolicFiniteDifferenceSolver2::*)()) &SimpleParabolicFiniteDifferenceSolver2::AssembleVector, 
            " "  )
        .def(
            "rGetIntermediateSolutions", 
            (::std::vector<std::pair<std::vector<double, std::allocator<double> >, double>, std::allocator<std::pair<std::vector<double, std::allocator<double> >, double> > > const &(SimpleParabolicFiniteDifferenceSolver2::*)()) &SimpleParabolicFiniteDifferenceSolver2::rGetIntermediateSolutions, 
            " "  )
        .def(
            "SetTargetTimeIncrement", 
            (void(SimpleParabolicFiniteDifferenceSolver2::*)(double)) &SimpleParabolicFiniteDifferenceSolver2::SetTargetTimeIncrement, 
            " " , py::arg("targetIncrement") )
        .def(
            "SetStartTime", 
            (void(SimpleParabolicFiniteDifferenceSolver2::*)(double)) &SimpleParabolicFiniteDifferenceSolver2::SetStartTime, 
            " " , py::arg("startTime") )
        .def(
            "SetEndTime", 
            (void(SimpleParabolicFiniteDifferenceSolver2::*)(double)) &SimpleParabolicFiniteDifferenceSolver2::SetEndTime, 
            " " , py::arg("endTime") )
        .def(
            "SetStoreIntermediateSolutions", 
            (void(SimpleParabolicFiniteDifferenceSolver2::*)(bool, unsigned int)) &SimpleParabolicFiniteDifferenceSolver2::SetStoreIntermediateSolutions, 
            " " , py::arg("store"), py::arg("frequency") = 1 )
        .def(
            "SetWriteIntermediateSolutions", 
            (void(SimpleParabolicFiniteDifferenceSolver2::*)(bool, unsigned int)) &SimpleParabolicFiniteDifferenceSolver2::SetWriteIntermediateSolutions, 
            " " , py::arg("write"), py::arg("frequency") = 1 )
        .def(
            "Solve", 
            (void(SimpleParabolicFiniteDifferenceSolver2::*)()) &SimpleParabolicFiniteDifferenceSolver2::Solve, 
            " "  )
    ;
}
