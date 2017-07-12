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
#include "SimpleLinearEllipticFiniteElementSolver.hpp"

#include "SimpleLinearEllipticFiniteElementSolver3.cppwg.hpp"

namespace py = pybind11;
typedef SimpleLinearEllipticFiniteElementSolver<3 > SimpleLinearEllipticFiniteElementSolver3;
;

class SimpleLinearEllipticFiniteElementSolver3_Overloads : public SimpleLinearEllipticFiniteElementSolver3{
    public:
    using SimpleLinearEllipticFiniteElementSolver3::SimpleLinearEllipticFiniteElementSolver;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteElementSolver3,
            Solve,
            );
    }

};
void register_SimpleLinearEllipticFiniteElementSolver3_class(py::module &m){
py::class_<SimpleLinearEllipticFiniteElementSolver3 , SimpleLinearEllipticFiniteElementSolver3_Overloads   >(m, "SimpleLinearEllipticFiniteElementSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleLinearEllipticFiniteElementSolver<3> >(*)()) &SimpleLinearEllipticFiniteElementSolver3::Create, 
            " "  )
        .def(
            "Solve", 
            (void(SimpleLinearEllipticFiniteElementSolver3::*)()) &SimpleLinearEllipticFiniteElementSolver3::Solve, 
            " "  )
    ;
}
