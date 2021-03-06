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
#include "CoupledLumpedSystemFiniteDifferenceSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef CoupledLumpedSystemFiniteDifferenceSolver<3 > CoupledLumpedSystemFiniteDifferenceSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
PYBIND11_MAKE_OPAQUE(Vec);
PYBIND11_MAKE_OPAQUE(Mat);
PYBIND11_MAKE_OPAQUE(TS);

class CoupledLumpedSystemFiniteDifferenceSolver3_Overloads : public CoupledLumpedSystemFiniteDifferenceSolver3{
    public:
    using CoupledLumpedSystemFiniteDifferenceSolver3::CoupledLumpedSystemFiniteDifferenceSolver;
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledLumpedSystemFiniteDifferenceSolver3,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledLumpedSystemFiniteDifferenceSolver3,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledLumpedSystemFiniteDifferenceSolver3,
            Solve,
            );
    }

};
void register_CoupledLumpedSystemFiniteDifferenceSolver3_class(py::module &m){
py::class_<CoupledLumpedSystemFiniteDifferenceSolver3 , CoupledLumpedSystemFiniteDifferenceSolver3_Overloads , std::shared_ptr<CoupledLumpedSystemFiniteDifferenceSolver3 >  , SimpleParabolicFiniteDifferenceSolver<3>  >(m, "CoupledLumpedSystemFiniteDifferenceSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CoupledLumpedSystemFiniteDifferenceSolver<3> >(*)()) &CoupledLumpedSystemFiniteDifferenceSolver3::Create, 
            " "  )
        .def(
            "AssembleMatrix", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)()) &CoupledLumpedSystemFiniteDifferenceSolver3::AssembleMatrix, 
            " "  )
        .def(
            "AssembleVector", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)()) &CoupledLumpedSystemFiniteDifferenceSolver3::AssembleVector, 
            " "  )
        .def(
            "ComputeRHSFunction", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)(::Vec const, ::Vec, ::TS)) &CoupledLumpedSystemFiniteDifferenceSolver3::ComputeRHSFunction, 
            " " , py::arg("currentGuess"), py::arg("dUdt"), py::arg("ts") )
        .def(
            "SetUseCoupling", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)(bool)) &CoupledLumpedSystemFiniteDifferenceSolver3::SetUseCoupling, 
            " " , py::arg("useCoupling") )
        .def(
            "Solve", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)()) &CoupledLumpedSystemFiniteDifferenceSolver3::Solve, 
            " "  )
    ;
}
