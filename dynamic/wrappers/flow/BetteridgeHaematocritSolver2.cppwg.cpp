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

#include "BetteridgeHaematocritSolver2.cppwg.hpp"

namespace py = pybind11;
typedef BetteridgeHaematocritSolver<2 > BetteridgeHaematocritSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class BetteridgeHaematocritSolver2_Overloads : public BetteridgeHaematocritSolver2{
    public:
    using BetteridgeHaematocritSolver2::BetteridgeHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            BetteridgeHaematocritSolver2,
            Calculate,
            );
    }

};
void register_BetteridgeHaematocritSolver2_class(py::module &m){
py::class_<BetteridgeHaematocritSolver2 , BetteridgeHaematocritSolver2_Overloads , std::shared_ptr<BetteridgeHaematocritSolver2 >  , AbstractHaematocritSolver<2>  >(m, "BetteridgeHaematocritSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<BetteridgeHaematocritSolver<2> >(*)()) &BetteridgeHaematocritSolver2::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(BetteridgeHaematocritSolver2::*)()) &BetteridgeHaematocritSolver2::Calculate, 
            " "  )
        .def(
            "SetExceptionOnFailedConverge", 
            (void(BetteridgeHaematocritSolver2::*)(bool)) &BetteridgeHaematocritSolver2::SetExceptionOnFailedConverge, 
            " " , py::arg("setException") )
        .def(
            "SetTHR", 
            (void(BetteridgeHaematocritSolver2::*)(::QDimensionless)) &BetteridgeHaematocritSolver2::SetTHR, 
            " " , py::arg("thr") )
        .def(
            "SetAlpha", 
            (void(BetteridgeHaematocritSolver2::*)(::QDimensionless)) &BetteridgeHaematocritSolver2::SetAlpha, 
            " " , py::arg("alpha") )
        .def(
            "SetHaematocrit", 
            (void(BetteridgeHaematocritSolver2::*)(::QDimensionless)) &BetteridgeHaematocritSolver2::SetHaematocrit, 
            " " , py::arg("haematocrit") )
        .def(
            "SetUseHigherConnectivityBranches", 
            (void(BetteridgeHaematocritSolver2::*)(bool)) &BetteridgeHaematocritSolver2::SetUseHigherConnectivityBranches, 
            " " , py::arg("useHighConnectivity") )
        .def(
            "SetTurnOffFungModel", 
            (void(BetteridgeHaematocritSolver2::*)(bool)) &BetteridgeHaematocritSolver2::SetTurnOffFungModel, 
            " " , py::arg("turnOffFungModel") )
        .def(
            "SetUseRandomSplittingModel", 
            (void(BetteridgeHaematocritSolver2::*)(bool)) &BetteridgeHaematocritSolver2::SetUseRandomSplittingModel, 
            " " , py::arg("useRandomSplittingModel") )
    ;
}
