#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "SimpleNonLinearEllipticFiniteElementSolver.hpp"

#include "SimpleNonLinearEllipticFiniteElementSolver2.cppwg.hpp"

namespace py = pybind11;
typedef SimpleNonLinearEllipticFiniteElementSolver<2 > SimpleNonLinearEllipticFiniteElementSolver2;
;

class SimpleNonLinearEllipticFiniteElementSolver2_Overloads : public SimpleNonLinearEllipticFiniteElementSolver2{
    public:
    using SimpleNonLinearEllipticFiniteElementSolver2::SimpleNonLinearEllipticFiniteElementSolver;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleNonLinearEllipticFiniteElementSolver2,
            Solve,
            );
    }

};
void register_SimpleNonLinearEllipticFiniteElementSolver2_class(py::module &m){
py::class_<SimpleNonLinearEllipticFiniteElementSolver2 , SimpleNonLinearEllipticFiniteElementSolver2_Overloads   >(m, "SimpleNonLinearEllipticFiniteElementSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleNonLinearEllipticFiniteElementSolver<2> >(*)()) &SimpleNonLinearEllipticFiniteElementSolver2::Create, 
            " "  )
        .def(
            "Solve", 
            (void(SimpleNonLinearEllipticFiniteElementSolver2::*)()) &SimpleNonLinearEllipticFiniteElementSolver2::Solve, 
            " "  )
    ;
}