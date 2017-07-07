#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"

#include "SimpleNonLinearEllipticFiniteDifferenceSolver3.cppwg.hpp"

namespace py = pybind11;
typedef SimpleNonLinearEllipticFiniteDifferenceSolver<3 > SimpleNonLinearEllipticFiniteDifferenceSolver3;
;

class SimpleNonLinearEllipticFiniteDifferenceSolver3_Overloads : public SimpleNonLinearEllipticFiniteDifferenceSolver3{
    public:
    using SimpleNonLinearEllipticFiniteDifferenceSolver3::SimpleNonLinearEllipticFiniteDifferenceSolver;
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleNonLinearEllipticFiniteDifferenceSolver3,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleNonLinearEllipticFiniteDifferenceSolver3,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleNonLinearEllipticFiniteDifferenceSolver3,
            Solve,
            );
    }

};
void register_SimpleNonLinearEllipticFiniteDifferenceSolver3_class(py::module &m){
py::class_<SimpleNonLinearEllipticFiniteDifferenceSolver3 , SimpleNonLinearEllipticFiniteDifferenceSolver3_Overloads   >(m, "SimpleNonLinearEllipticFiniteDifferenceSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleNonLinearEllipticFiniteDifferenceSolver<3> >(*)()) &SimpleNonLinearEllipticFiniteDifferenceSolver3::Create, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(SimpleNonLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleNonLinearEllipticFiniteDifferenceSolver3::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(SimpleNonLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleNonLinearEllipticFiniteDifferenceSolver3::AssembleVector, 
            " "  )
        .def(
            "Solve", 
            (void(SimpleNonLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleNonLinearEllipticFiniteDifferenceSolver3::Solve, 
            " "  )
    ;
}
