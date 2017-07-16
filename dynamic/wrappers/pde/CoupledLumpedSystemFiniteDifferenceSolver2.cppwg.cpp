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
#include "CoupledLumpedSystemFiniteDifferenceSolver.hpp"

#include "CoupledLumpedSystemFiniteDifferenceSolver2.cppwg.hpp"

namespace py = pybind11;
typedef CoupledLumpedSystemFiniteDifferenceSolver<2 > CoupledLumpedSystemFiniteDifferenceSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

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
            "SetUseCoupling", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver2::*)(bool)) &CoupledLumpedSystemFiniteDifferenceSolver2::SetUseCoupling, 
            " " , py::arg("useCoupling") )
        .def(
            "Solve", 
            (void(CoupledLumpedSystemFiniteDifferenceSolver2::*)()) &CoupledLumpedSystemFiniteDifferenceSolver2::Solve, 
            " "  )
    ;
}
