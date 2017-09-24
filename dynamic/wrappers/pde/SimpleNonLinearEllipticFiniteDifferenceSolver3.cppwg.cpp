#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <petsc/private/vecimpl.h>
#include <petsc/private/matimpl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "vtkPolyData.h"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"

#include "PythonObjectConverters.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef SimpleNonLinearEllipticFiniteDifferenceSolver<3 > SimpleNonLinearEllipticFiniteDifferenceSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
PYBIND11_MAKE_OPAQUE(Vec);
PYBIND11_MAKE_OPAQUE(Mat);

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
py::class_<SimpleNonLinearEllipticFiniteDifferenceSolver3 , SimpleNonLinearEllipticFiniteDifferenceSolver3_Overloads , std::shared_ptr<SimpleNonLinearEllipticFiniteDifferenceSolver3 >  , AbstractFiniteDifferenceSolverBase<3>  >(m, "SimpleNonLinearEllipticFiniteDifferenceSolver3")
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
