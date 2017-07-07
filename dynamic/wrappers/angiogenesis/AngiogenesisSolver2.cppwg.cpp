#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AngiogenesisSolver.hpp"

#include "AngiogenesisSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AngiogenesisSolver<2 > AngiogenesisSolver2;
;

class AngiogenesisSolver2_Overloads : public AngiogenesisSolver2{
    public:
    using AngiogenesisSolver2::AngiogenesisSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver2,
            Increment,
            );
    }
    void DoSprouting() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver2,
            DoSprouting,
            );
    }
    void DoAnastamosis() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver2,
            DoAnastamosis,
            );
    }
    void UpdateNodalPositions(bool sprouting) override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver2,
            UpdateNodalPositions,
            sprouting);
    }

};
void register_AngiogenesisSolver2_class(py::module &m){
py::class_<AngiogenesisSolver2 , AngiogenesisSolver2_Overloads   >(m, "AngiogenesisSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AngiogenesisSolver<2> >(*)()) &AngiogenesisSolver2::Create, 
            " "  )
        .def(
            "Increment", 
            (void(AngiogenesisSolver2::*)()) &AngiogenesisSolver2::Increment, 
            " "  )
        .def(
            "IsSproutingRuleSet", 
            (bool(AngiogenesisSolver2::*)()) &AngiogenesisSolver2::IsSproutingRuleSet, 
            " "  )
        .def(
            "SetDoAnastomosis", 
            (void(AngiogenesisSolver2::*)(bool)) &AngiogenesisSolver2::SetDoAnastomosis, 
            " " , py::arg("doAnastomosis") )
        .def(
            "Run", 
            (void(AngiogenesisSolver2::*)(bool)) &AngiogenesisSolver2::Run, 
            " " , py::arg("writeOutput") = false )
        .def(
            "SetAnastamosisRadius", 
            (void(AngiogenesisSolver2::*)(::QLength)) &AngiogenesisSolver2::SetAnastamosisRadius, 
            " " , py::arg("radius") )
        .def(
            "SetBoundingDomain", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<Part<2> >)) &AngiogenesisSolver2::SetBoundingDomain, 
            " " , py::arg("pDomain") )
        .def(
            "SetCellPopulation", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<AbstractCellPopulation<2, 2> >, ::QLength)) &AngiogenesisSolver2::SetCellPopulation, 
            " " , py::arg("pCellPopulation"), py::arg("cellPopulationReferenceLength") )
        .def(
            "SetMigrationRule", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<AbstractMigrationRule<2> >)) &AngiogenesisSolver2::SetMigrationRule, 
            " " , py::arg("pMigrationRule") )
        .def(
            "SetOutputFileHandler", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<OutputFileHandler>)) &AngiogenesisSolver2::SetOutputFileHandler, 
            " " , py::arg("pHandler") )
        .def(
            "SetSproutingRule", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<AbstractSproutingRule<2> >)) &AngiogenesisSolver2::SetSproutingRule, 
            " " , py::arg("pSproutingRule") )
        .def(
            "SetVesselGridCalculator", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<GridCalculator<2> >)) &AngiogenesisSolver2::SetVesselGridCalculator, 
            " " , py::arg("pVesselGrid") )
        .def(
            "SetVesselNetwork", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AngiogenesisSolver2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
    ;
}
