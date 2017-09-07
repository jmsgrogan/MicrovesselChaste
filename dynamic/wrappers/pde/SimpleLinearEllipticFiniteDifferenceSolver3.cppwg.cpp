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

#include "PythonObjectConverters.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef SimpleLinearEllipticFiniteDifferenceSolver<3 > SimpleLinearEllipticFiniteDifferenceSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::shared_ptr<LinearSystem> _std_shared_ptr_lt_LinearSystem_gt_;

class SimpleLinearEllipticFiniteDifferenceSolver3_Overloads : public SimpleLinearEllipticFiniteDifferenceSolver3{
    public:
    using SimpleLinearEllipticFiniteDifferenceSolver3::SimpleLinearEllipticFiniteDifferenceSolver;
    void AddDiscreteTermsToMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver3,
            AddDiscreteTermsToMatrix,
            );
    }
    void AddDiscreteTermsToRhs() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver3,
            AddDiscreteTermsToRhs,
            );
    }
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver3,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver3,
            AssembleVector,
            );
    }
    ::std::shared_ptr<LinearSystem> GetLinearSystem() override {
        PYBIND11_OVERLOAD(
            _std_shared_ptr_lt_LinearSystem_gt_,
            SimpleLinearEllipticFiniteDifferenceSolver3,
            GetLinearSystem,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver3,
            Setup,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver3,
            Solve,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            SimpleLinearEllipticFiniteDifferenceSolver3,
            Update,
            );
    }

};
void register_SimpleLinearEllipticFiniteDifferenceSolver3_class(py::module &m){
py::class_<SimpleLinearEllipticFiniteDifferenceSolver3 , SimpleLinearEllipticFiniteDifferenceSolver3_Overloads , std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver3 >  , AbstractFiniteDifferenceSolverBase<3>  >(m, "SimpleLinearEllipticFiniteDifferenceSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<3> >(*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::Create, 
            " "  )
        .def(
            "AddDiscreteTermsToMatrix", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::AddDiscreteTermsToMatrix, 
            " "  )
        .def(
            "AddDiscreteTermsToRhs", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::AddDiscreteTermsToRhs, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::AssembleVector, 
            " "  )
        .def(
            "GetLinearSystem", 
            (::std::shared_ptr<LinearSystem>(SimpleLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::GetLinearSystem, 
            " "  )
        .def(
            "Setup", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::Setup, 
            " "  )
        .def(
            "Solve", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::Solve, 
            " "  )
        .def(
            "Update", 
            (void(SimpleLinearEllipticFiniteDifferenceSolver3::*)()) &SimpleLinearEllipticFiniteDifferenceSolver3::Update, 
            " "  )
    ;
}
