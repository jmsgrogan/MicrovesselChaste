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
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"

#include "SimpleNonLinearEllipticFiniteDifferenceSolver2.cppwg.hpp"

namespace py = pybind11;
typedef SimpleNonLinearEllipticFiniteDifferenceSolver<2 > SimpleNonLinearEllipticFiniteDifferenceSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class SimpleNonLinearEllipticFiniteDifferenceSolver2_Overloads : public SimpleNonLinearEllipticFiniteDifferenceSolver2{
    public:
    using SimpleNonLinearEllipticFiniteDifferenceSolver2::SimpleNonLinearEllipticFiniteDifferenceSolver;
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleNonLinearEllipticFiniteDifferenceSolver2,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleNonLinearEllipticFiniteDifferenceSolver2,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleNonLinearEllipticFiniteDifferenceSolver2,
            Solve,
            );
    }

};
void register_SimpleNonLinearEllipticFiniteDifferenceSolver2_class(py::module &m){
py::class_<SimpleNonLinearEllipticFiniteDifferenceSolver2 , SimpleNonLinearEllipticFiniteDifferenceSolver2_Overloads , std::shared_ptr<SimpleNonLinearEllipticFiniteDifferenceSolver2 >  , AbstractFiniteDifferenceSolverBase<2>  >(m, "SimpleNonLinearEllipticFiniteDifferenceSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleNonLinearEllipticFiniteDifferenceSolver<2> >(*)()) &SimpleNonLinearEllipticFiniteDifferenceSolver2::Create, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(SimpleNonLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleNonLinearEllipticFiniteDifferenceSolver2::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(SimpleNonLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleNonLinearEllipticFiniteDifferenceSolver2::AssembleVector, 
            " "  )
        .def(
            "Solve", 
            (void(SimpleNonLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleNonLinearEllipticFiniteDifferenceSolver2::Solve, 
            " "  )
    ;
}
