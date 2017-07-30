#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <petsc/private/vecimpl.h>
#include <petsc/private/matimpl.h>
#include <petsc/private/tsimpl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "vtkPolyData.h"
#include "CoupledLumpedSystemFiniteElementSolver.hpp"

#include "CoupledLumpedSystemFiniteElementSolver3.cppwg.hpp"

namespace py = pybind11;
typedef CoupledLumpedSystemFiniteElementSolver<3 > CoupledLumpedSystemFiniteElementSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
PYBIND11_MAKE_OPAQUE(Vec);
PYBIND11_MAKE_OPAQUE(Mat);
PYBIND11_MAKE_OPAQUE(TS);

class CoupledLumpedSystemFiniteElementSolver3_Overloads : public CoupledLumpedSystemFiniteElementSolver3{
    public:
    using CoupledLumpedSystemFiniteElementSolver3::CoupledLumpedSystemFiniteElementSolver;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledLumpedSystemFiniteElementSolver3,
            Solve,
            );
    }

};
void register_CoupledLumpedSystemFiniteElementSolver3_class(py::module &m){
py::class_<CoupledLumpedSystemFiniteElementSolver3 , CoupledLumpedSystemFiniteElementSolver3_Overloads , std::shared_ptr<CoupledLumpedSystemFiniteElementSolver3 >  , AbstractFiniteElementSolverBase<3>  >(m, "CoupledLumpedSystemFiniteElementSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CoupledLumpedSystemFiniteElementSolver<3> >(*)()) &CoupledLumpedSystemFiniteElementSolver3::Create, 
            " "  )
        .def(
            "rGetIntermediateSolutions", 
            (::std::vector<std::pair<std::vector<double, std::allocator<double> >, double>, std::allocator<std::pair<std::vector<double, std::allocator<double> >, double> > > const &(CoupledLumpedSystemFiniteElementSolver3::*)()) &CoupledLumpedSystemFiniteElementSolver3::rGetIntermediateSolutions, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "SetTargetTimeIncrement", 
            (void(CoupledLumpedSystemFiniteElementSolver3::*)(double)) &CoupledLumpedSystemFiniteElementSolver3::SetTargetTimeIncrement, 
            " " , py::arg("targetIncrement") )
        .def(
            "SetStartTime", 
            (void(CoupledLumpedSystemFiniteElementSolver3::*)(double)) &CoupledLumpedSystemFiniteElementSolver3::SetStartTime, 
            " " , py::arg("startTime") )
        .def(
            "SetEndTime", 
            (void(CoupledLumpedSystemFiniteElementSolver3::*)(double)) &CoupledLumpedSystemFiniteElementSolver3::SetEndTime, 
            " " , py::arg("endTime") )
        .def(
            "SetInitialGuess", 
            (void(CoupledLumpedSystemFiniteElementSolver3::*)(::std::vector<double, std::allocator<double> > const &)) &CoupledLumpedSystemFiniteElementSolver3::SetInitialGuess, 
            " " , py::arg("rInitialGuess") )
        .def(
            "SetStoreIntermediateSolutions", 
            (void(CoupledLumpedSystemFiniteElementSolver3::*)(bool, unsigned int)) &CoupledLumpedSystemFiniteElementSolver3::SetStoreIntermediateSolutions, 
            " " , py::arg("store"), py::arg("frequency") = 1 )
        .def(
            "SetWriteIntermediateSolutions", 
            (void(CoupledLumpedSystemFiniteElementSolver3::*)(bool, unsigned int)) &CoupledLumpedSystemFiniteElementSolver3::SetWriteIntermediateSolutions, 
            " " , py::arg("write"), py::arg("frequency") = 1 )
        .def(
            "SetUseCoupling", 
            (void(CoupledLumpedSystemFiniteElementSolver3::*)(bool)) &CoupledLumpedSystemFiniteElementSolver3::SetUseCoupling, 
            " " , py::arg("useCoupling") )
        .def(
            "Solve", 
            (void(CoupledLumpedSystemFiniteElementSolver3::*)()) &CoupledLumpedSystemFiniteElementSolver3::Solve, 
            " "  )
    ;
}
