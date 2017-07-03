#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractFiniteElementSolverBase.hpp"

#include "AbstractFiniteElementSolverBase3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractFiniteElementSolverBase<3 > AbstractFiniteElementSolverBase3;
;

class AbstractFiniteElementSolverBase3_Overloads : public AbstractFiniteElementSolverBase3{
    public:
    using AbstractFiniteElementSolverBase3::AbstractFiniteElementSolverBase;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteElementSolverBase3,
            Solve,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteElementSolverBase3,
            Setup,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteElementSolverBase3,
            Update,
            );
    }

};
void register_AbstractFiniteElementSolverBase3_class(py::module &m){
py::class_<AbstractFiniteElementSolverBase3 , AbstractFiniteElementSolverBase3_Overloads   >(m, "AbstractFiniteElementSolverBase3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AbstractFiniteElementSolverBase<3> >(*)()) &AbstractFiniteElementSolverBase3::Create, 
            " " )
        .def(
            "Solve", 
            (void(AbstractFiniteElementSolverBase3::*)()) &AbstractFiniteElementSolverBase3::Solve, 
            " " )
        .def(
            "SetGuess", 
            (void(AbstractFiniteElementSolverBase3::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractFiniteElementSolverBase3::SetGuess, 
            " " , py::arg("guess"))
        .def(
            "SetUseSimpleNetonSolver", 
            (void(AbstractFiniteElementSolverBase3::*)(bool)) &AbstractFiniteElementSolverBase3::SetUseSimpleNetonSolver, 
            " " , py::arg("useNewton"))
        .def(
            "Setup", 
            (void(AbstractFiniteElementSolverBase3::*)()) &AbstractFiniteElementSolverBase3::Setup, 
            " " )
        .def(
            "Update", 
            (void(AbstractFiniteElementSolverBase3::*)()) &AbstractFiniteElementSolverBase3::Update, 
            " " )
    ;
}
