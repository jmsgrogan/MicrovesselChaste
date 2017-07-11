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

#include "SimpleParabolicFiniteDifferenceSolver3.cppwg.hpp"

namespace py = pybind11;
typedef SimpleParabolicFiniteDifferenceSolver<3 > SimpleParabolicFiniteDifferenceSolver3;
;

class SimpleParabolicFiniteDifferenceSolver3_Overloads : public SimpleParabolicFiniteDifferenceSolver3{
    public:
    using SimpleParabolicFiniteDifferenceSolver3::SimpleParabolicFiniteDifferenceSolver;
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleParabolicFiniteDifferenceSolver3,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleParabolicFiniteDifferenceSolver3,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleParabolicFiniteDifferenceSolver3,
            Solve,
            );
    }

};
void register_SimpleParabolicFiniteDifferenceSolver3_class(py::module &m){
py::class_<SimpleParabolicFiniteDifferenceSolver3 , SimpleParabolicFiniteDifferenceSolver3_Overloads   >(m, "SimpleParabolicFiniteDifferenceSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleParabolicFiniteDifferenceSolver<3> >(*)()) &SimpleParabolicFiniteDifferenceSolver3::Create, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(SimpleParabolicFiniteDifferenceSolver3::*)()) &SimpleParabolicFiniteDifferenceSolver3::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(SimpleParabolicFiniteDifferenceSolver3::*)()) &SimpleParabolicFiniteDifferenceSolver3::AssembleVector, 
            " "  )
        .def(
            "rGetIntermediateSolutions", 
            (::std::vector<std::pair<std::vector<double, std::allocator<double> >, double>, std::allocator<std::pair<std::vector<double, std::allocator<double> >, double> > > const &(SimpleParabolicFiniteDifferenceSolver3::*)()) &SimpleParabolicFiniteDifferenceSolver3::rGetIntermediateSolutions, 
            " "  )
        .def(
            "SetTargetTimeIncrement", 
            (void(SimpleParabolicFiniteDifferenceSolver3::*)(double)) &SimpleParabolicFiniteDifferenceSolver3::SetTargetTimeIncrement, 
            " " , py::arg("targetIncrement") )
        .def(
            "SetStartTime", 
            (void(SimpleParabolicFiniteDifferenceSolver3::*)(double)) &SimpleParabolicFiniteDifferenceSolver3::SetStartTime, 
            " " , py::arg("startTime") )
        .def(
            "SetEndTime", 
            (void(SimpleParabolicFiniteDifferenceSolver3::*)(double)) &SimpleParabolicFiniteDifferenceSolver3::SetEndTime, 
            " " , py::arg("endTime") )
        .def(
            "SetStoreIntermediateSolutions", 
            (void(SimpleParabolicFiniteDifferenceSolver3::*)(bool, unsigned int)) &SimpleParabolicFiniteDifferenceSolver3::SetStoreIntermediateSolutions, 
            " " , py::arg("store"), py::arg("frequency") = 1 )
        .def(
            "SetWriteIntermediateSolutions", 
            (void(SimpleParabolicFiniteDifferenceSolver3::*)(bool, unsigned int)) &SimpleParabolicFiniteDifferenceSolver3::SetWriteIntermediateSolutions, 
            " " , py::arg("write"), py::arg("frequency") = 1 )
        .def(
            "Solve", 
            (void(SimpleParabolicFiniteDifferenceSolver3::*)()) &SimpleParabolicFiniteDifferenceSolver3::Solve, 
            " "  )
    ;
}