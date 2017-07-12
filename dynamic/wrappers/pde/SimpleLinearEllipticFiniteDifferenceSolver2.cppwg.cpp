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
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"

#include "SimpleLinearEllipticFiniteDifferenceSolver2.cppwg.hpp"

namespace py = pybind11;
typedef SimpleLinearEllipticFiniteDifferenceSolver<2 > SimpleLinearEllipticFiniteDifferenceSolver2;
;
typedef ::std::shared_ptr<LinearSystem> _std_shared_ptrLinearSystem;

class SimpleLinearEllipticFiniteDifferenceSolver2_Overloads : public SimpleLinearEllipticFiniteDifferenceSolver2{
    public:
    using SimpleLinearEllipticFiniteDifferenceSolver2::SimpleLinearEllipticFiniteDifferenceSolver;
    void AddDiscreteTermsToMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver2,
            AddDiscreteTermsToMatrix,
            );
    }
    void AddDiscreteTermsToRhs() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver2,
            AddDiscreteTermsToRhs,
            );
    }
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver2,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver2,
            AssembleVector,
            );
    }
    ::std::shared_ptr<LinearSystem> GetLinearSystem() override {
        PYBIND11_OVERLOAD(
            _std_shared_ptrLinearSystem,
            SimpleLinearEllipticFiniteDifferenceSolver2,
            GetLinearSystem,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver2,
            Setup,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver2,
            Solve,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver2,
            Update,
            );
    }

};
void register_SimpleLinearEllipticFiniteDifferenceSolver2_class(py::module &m){
py::class_<SimpleLinearEllipticFiniteDifferenceSolver2 , SimpleLinearEllipticFiniteDifferenceSolver2_Overloads   >(m, "SimpleLinearEllipticFiniteDifferenceSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<2> >(*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::Create, 
            " "  )
        .def(
            "AddDiscreteTermsToMatrix", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::AddDiscreteTermsToMatrix, 
            " "  )
        .def(
            "AddDiscreteTermsToRhs", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::AddDiscreteTermsToRhs, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::AssembleVector, 
            " "  )
        .def(
            "GetLinearSystem", 
            (::std::shared_ptr<LinearSystem>(SimpleLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::GetLinearSystem, 
            " "  )
        .def(
            "Setup", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::Setup, 
            " "  )
        .def(
            "Solve", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::Solve, 
            " "  )
        .def(
            "Update", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver2::*)()) &SimpleLinearEllipticFiniteDifferenceSolver2::Update, 
            " "  )
    ;
}
