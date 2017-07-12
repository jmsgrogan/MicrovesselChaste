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
#include "AbstractFiniteElementSolverBase.hpp"

#include "AbstractFiniteElementSolverBase2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractFiniteElementSolverBase<2 > AbstractFiniteElementSolverBase2;
;

class AbstractFiniteElementSolverBase2_Overloads : public AbstractFiniteElementSolverBase2{
    public:
    using AbstractFiniteElementSolverBase2::AbstractFiniteElementSolverBase;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteElementSolverBase2,
            Solve,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteElementSolverBase2,
            Setup,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteElementSolverBase2,
            Update,
            );
    }

};
void register_AbstractFiniteElementSolverBase2_class(py::module &m){
py::class_<AbstractFiniteElementSolverBase2 , AbstractFiniteElementSolverBase2_Overloads   >(m, "AbstractFiniteElementSolverBase2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AbstractFiniteElementSolverBase<2> >(*)()) &AbstractFiniteElementSolverBase2::Create, 
            " "  )
        .def(
            "Solve", 
            (void(AbstractFiniteElementSolverBase2::*)()) &AbstractFiniteElementSolverBase2::Solve, 
            " "  )
        .def(
            "SetGuess", 
            (void(AbstractFiniteElementSolverBase2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractFiniteElementSolverBase2::SetGuess, 
            " " , py::arg("guess") )
        .def(
            "SetUseSimpleNetonSolver", 
            (void(AbstractFiniteElementSolverBase2::*)(bool)) &AbstractFiniteElementSolverBase2::SetUseSimpleNetonSolver, 
            " " , py::arg("useNewton") )
        .def(
            "Setup", 
            (void(AbstractFiniteElementSolverBase2::*)()) &AbstractFiniteElementSolverBase2::Setup, 
            " "  )
        .def(
            "Update", 
            (void(AbstractFiniteElementSolverBase2::*)()) &AbstractFiniteElementSolverBase2::Update, 
            " "  )
    ;
}
