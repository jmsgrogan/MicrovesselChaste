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
#include "BetteridgeHaematocritSolver.hpp"

#include "PythonObjectConverters.hpp"
#include "BetteridgeHaematocritSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef BetteridgeHaematocritSolver<3 > BetteridgeHaematocritSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class BetteridgeHaematocritSolver3_Overloads : public BetteridgeHaematocritSolver3{
    public:
    using BetteridgeHaematocritSolver3::BetteridgeHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            BetteridgeHaematocritSolver3,
            Calculate,
            );
    }

};
void register_BetteridgeHaematocritSolver3_class(py::module &m){
py::class_<BetteridgeHaematocritSolver3 , BetteridgeHaematocritSolver3_Overloads , std::shared_ptr<BetteridgeHaematocritSolver3 >  , AbstractHaematocritSolver<3>  >(m, "BetteridgeHaematocritSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<BetteridgeHaematocritSolver<3> >(*)()) &BetteridgeHaematocritSolver3::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(BetteridgeHaematocritSolver3::*)()) &BetteridgeHaematocritSolver3::Calculate, 
            " "  )
        .def(
            "SetExceptionOnFailedConverge", 
            (void(BetteridgeHaematocritSolver3::*)(bool)) &BetteridgeHaematocritSolver3::SetExceptionOnFailedConverge, 
            " " , py::arg("setException") )
        .def(
            "SetTHR", 
            (void(BetteridgeHaematocritSolver3::*)(::QDimensionless)) &BetteridgeHaematocritSolver3::SetTHR, 
            " " , py::arg("thr") )
        .def(
            "SetAlpha", 
            (void(BetteridgeHaematocritSolver3::*)(::QDimensionless)) &BetteridgeHaematocritSolver3::SetAlpha, 
            " " , py::arg("alpha") )
        .def(
            "SetHaematocrit", 
            (void(BetteridgeHaematocritSolver3::*)(::QDimensionless)) &BetteridgeHaematocritSolver3::SetHaematocrit, 
            " " , py::arg("haematocrit") )
        .def(
            "SetUseHigherConnectivityBranches", 
            (void(BetteridgeHaematocritSolver3::*)(bool)) &BetteridgeHaematocritSolver3::SetUseHigherConnectivityBranches, 
            " " , py::arg("useHighConnectivity") )
        .def(
            "SetTurnOffFungModel", 
            (void(BetteridgeHaematocritSolver3::*)(bool)) &BetteridgeHaematocritSolver3::SetTurnOffFungModel, 
            " " , py::arg("turnOffFungModel") )
        .def(
            "SetUseRandomSplittingModel", 
            (void(BetteridgeHaematocritSolver3::*)(bool)) &BetteridgeHaematocritSolver3::SetUseRandomSplittingModel, 
            " " , py::arg("useRandomSplittingModel") )
    ;
}
