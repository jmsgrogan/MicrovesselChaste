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
#include "CoupledLumpedSystemFiniteDifferenceSolver.hpp"

#include "PythonObjectConverters.hpp"
#include "CoupledLumpedSystemFiniteDifferenceSolver2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef CoupledLumpedSystemFiniteDifferenceSolver<2 > CoupledLumpedSystemFiniteDifferenceSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
PYBIND11_MAKE_OPAQUE(Vec);
PYBIND11_MAKE_OPAQUE(Mat);
PYBIND11_MAKE_OPAQUE(TS);

class CoupledLumpedSystemFiniteDifferenceSolver2_Overloads : public CoupledLumpedSystemFiniteDifferenceSolver2{
    public:
    using CoupledLumpedSystemFiniteDifferenceSolver2::CoupledLumpedSystemFiniteDifferenceSolver;
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledLumpedSystemFiniteDifferenceSolver2,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledLumpedSystemFiniteDifferenceSolver2,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledLumpedSystemFiniteDifferenceSolver2,
            Solve,
            );
    }

};
void register_CoupledLumpedSystemFiniteDifferenceSolver2_class(py::module &m){
py::class_<CoupledLumpedSystemFiniteDifferenceSolver2 , CoupledLumpedSystemFiniteDifferenceSolver2_Overloads , std::shared_ptr<CoupledLumpedSystemFiniteDifferenceSolver2 >  , SimpleParabolicFiniteDifferenceSolver<2>  >(m, "CoupledLumpedSystemFiniteDifferenceSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CoupledLumpedSystemFiniteDifferenceSolver<2> >(*)()) &CoupledLumpedSystemFiniteDifferenceSolver2::Create, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver2::*)()) &CoupledLumpedSystemFiniteDifferenceSolver2::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver2::*)()) &CoupledLumpedSystemFiniteDifferenceSolver2::AssembleVector, 
            " "  )
        .def(
            "ComputeRHSFunction", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver2::*)(::Vec const, ::Vec, ::TS)) &CoupledLumpedSystemFiniteDifferenceSolver2::ComputeRHSFunction, 
            " " , py::arg("currentGuess"), py::arg("dUdt"), py::arg("ts") )
        .def(
            "SetUseCoupling", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver2::*)(bool)) &CoupledLumpedSystemFiniteDifferenceSolver2::SetUseCoupling, 
            " " , py::arg("useCoupling") )
        .def(
            "Solve", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver2::*)()) &CoupledLumpedSystemFiniteDifferenceSolver2::Solve, 
            " "  )
    ;
}
