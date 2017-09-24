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
#include "AbstractFiniteDifferenceSolverBase.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractFiniteDifferenceSolverBase2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractFiniteDifferenceSolverBase<2 > AbstractFiniteDifferenceSolverBase2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
PYBIND11_MAKE_OPAQUE(Vec);
PYBIND11_MAKE_OPAQUE(Mat);

class AbstractFiniteDifferenceSolverBase2_Overloads : public AbstractFiniteDifferenceSolverBase2{
    public:
    using AbstractFiniteDifferenceSolverBase2::AbstractFiniteDifferenceSolverBase;
    void AddDiscreteTermsToMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            AddDiscreteTermsToMatrix,
            );
    }
    void AddDiscreteTermsToRhs() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            AddDiscreteTermsToRhs,
            );
    }
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractFiniteDifferenceSolverBase2,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractFiniteDifferenceSolverBase2,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            Solve,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            Setup,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            Update,
            );
    }

};
void register_AbstractFiniteDifferenceSolverBase2_class(py::module &m){
py::class_<AbstractFiniteDifferenceSolverBase2 , AbstractFiniteDifferenceSolverBase2_Overloads , std::shared_ptr<AbstractFiniteDifferenceSolverBase2 >  , AbstractRegularGridDiscreteContinuumSolver<2>  >(m, "AbstractFiniteDifferenceSolverBase2")
        .def(
            "AddDiscreteTermsToMatrix", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::AddDiscreteTermsToMatrix, 
            " "  )
        .def(
            "AddDiscreteTermsToRhs", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::AddDiscreteTermsToRhs, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::AssembleVector, 
            " "  )
        .def(
            "Solve", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::Solve, 
            " "  )
        .def(
            "Setup", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::Setup, 
            " "  )
        .def(
            "Update", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::Update, 
            " "  )
        .def(
            "UpdateBoundaryConditionsEachSolve", 
            (void(AbstractFiniteDifferenceSolverBase2::*)(bool)) &AbstractFiniteDifferenceSolverBase2::UpdateBoundaryConditionsEachSolve, 
            " " , py::arg("doUpdate") )
    ;
}
