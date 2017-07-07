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

#include "SimpleNonLinearEllipticFiniteElementSolver3.cppwg.hpp"

namespace py = pybind11;
typedef SimpleNonLinearEllipticFiniteElementSolver<3 > SimpleNonLinearEllipticFiniteElementSolver3;
;

class SimpleNonLinearEllipticFiniteElementSolver3_Overloads : public SimpleNonLinearEllipticFiniteElementSolver3{
    public:
    using SimpleNonLinearEllipticFiniteElementSolver3::SimpleNonLinearEllipticFiniteElementSolver;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleNonLinearEllipticFiniteElementSolver3,
            Solve,
            );
    }

};
void register_SimpleNonLinearEllipticFiniteElementSolver3_class(py::module &m){
py::class_<SimpleNonLinearEllipticFiniteElementSolver3 , SimpleNonLinearEllipticFiniteElementSolver3_Overloads   >(m, "SimpleNonLinearEllipticFiniteElementSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleNonLinearEllipticFiniteElementSolver<3> >(*)()) &SimpleNonLinearEllipticFiniteElementSolver3::Create, 
            " "  )
        .def(
            "Solve", 
            (void(SimpleNonLinearEllipticFiniteElementSolver3::*)()) &SimpleNonLinearEllipticFiniteElementSolver3::Solve, 
            " "  )
    ;
}
