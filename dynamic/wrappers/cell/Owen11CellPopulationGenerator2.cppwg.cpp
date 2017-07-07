#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Owen11CellPopulationGenerator.hpp"

#include "Owen11CellPopulationGenerator2.cppwg.hpp"

namespace py = pybind11;
typedef Owen11CellPopulationGenerator<2 > Owen11CellPopulationGenerator2;
;

void register_Owen11CellPopulationGenerator2_class(py::module &m){
py::class_<Owen11CellPopulationGenerator2    >(m, "Owen11CellPopulationGenerator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<Owen11CellPopulationGenerator<2> >(*)()) &Owen11CellPopulationGenerator2::Create, 
            " "  )
        .def(
            "SetAddTumour", 
            (void(Owen11CellPopulationGenerator2::*)(bool)) &Owen11CellPopulationGenerator2::SetAddTumour, 
            " " , py::arg("addTumour") )
        .def(
            "SetCellFraction", 
            (void(Owen11CellPopulationGenerator2::*)(double)) &Owen11CellPopulationGenerator2::SetCellFraction, 
            " " , py::arg("cellFraction") )
        .def(
            "SetVesselNetwork", 
            (void(Owen11CellPopulationGenerator2::*)(::std::shared_ptr<VesselNetwork<2> >)) &Owen11CellPopulationGenerator2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetGridCalculator", 
            (void(Owen11CellPopulationGenerator2::*)(::std::shared_ptr<GridCalculator<2> >)) &Owen11CellPopulationGenerator2::SetGridCalculator, 
            " " , py::arg("pGrid") )
        .def(
            "SetReferenceLengthScale", 
            (void(Owen11CellPopulationGenerator2::*)(::QLength)) &Owen11CellPopulationGenerator2::SetReferenceLengthScale, 
            " " , py::arg("lengthScale") )
        .def(
            "SetTumourRadius", 
            (void(Owen11CellPopulationGenerator2::*)(::QLength)) &Owen11CellPopulationGenerator2::SetTumourRadius, 
            " " , py::arg("tumourRadius") )
        .def(
            "Update", 
            (::std::shared_ptr<CaBasedCellPopulation<2> >(Owen11CellPopulationGenerator2::*)()) &Owen11CellPopulationGenerator2::Update, 
            " "  )
    ;
}
