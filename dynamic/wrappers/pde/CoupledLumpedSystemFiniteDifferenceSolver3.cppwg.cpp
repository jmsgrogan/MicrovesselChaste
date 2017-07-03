#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "CoupledLumpedSystemFiniteDifferenceSolver.hpp"

#include "CoupledLumpedSystemFiniteDifferenceSolver3.cppwg.hpp"

namespace py = pybind11;
typedef CoupledLumpedSystemFiniteDifferenceSolver<3 > CoupledLumpedSystemFiniteDifferenceSolver3;
;

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
py::class_<CoupledLumpedSystemFiniteDifferenceSolver3 , CoupledLumpedSystemFiniteDifferenceSolver3_Overloads   >(m, "CoupledLumpedSystemFiniteDifferenceSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CoupledLumpedSystemFiniteDifferenceSolver<3> >(*)()) &CoupledLumpedSystemFiniteDifferenceSolver3::Create, 
            " " )
        .def(
            "AssembleMatrix", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)()) &CoupledLumpedSystemFiniteDifferenceSolver3::AssembleMatrix, 
            " " )
        .def(
            "AssembleVector", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)()) &CoupledLumpedSystemFiniteDifferenceSolver3::AssembleVector, 
            " " )
        .def(
            "SetUseCoupling", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)(bool)) &CoupledLumpedSystemFiniteDifferenceSolver3::SetUseCoupling, 
            " " , py::arg("useCoupling"))
        .def(
            "Solve", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver3::*)()) &CoupledLumpedSystemFiniteDifferenceSolver3::Solve, 
            " " )
    ;
}
