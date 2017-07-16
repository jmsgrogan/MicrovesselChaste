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
#include "Owen11CellPopulationGenerator.hpp"

#include "Owen11CellPopulationGenerator3.cppwg.hpp"

namespace py = pybind11;
typedef Owen11CellPopulationGenerator<3 > Owen11CellPopulationGenerator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_Owen11CellPopulationGenerator3_class(py::module &m){
py::class_<Owen11CellPopulationGenerator3  , std::shared_ptr<Owen11CellPopulationGenerator3 >   >(m, "Owen11CellPopulationGenerator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<Owen11CellPopulationGenerator<3> >(*)()) &Owen11CellPopulationGenerator3::Create, 
            " "  )
        .def(
            "SetAddTumour", 
            (void(Owen11CellPopulationGenerator3::*)(bool)) &Owen11CellPopulationGenerator3::SetAddTumour, 
            " " , py::arg("addTumour") )
        .def(
            "SetCellFraction", 
            (void(Owen11CellPopulationGenerator3::*)(double)) &Owen11CellPopulationGenerator3::SetCellFraction, 
            " " , py::arg("cellFraction") )
        .def(
            "SetVesselNetwork", 
            (void(Owen11CellPopulationGenerator3::*)(::std::shared_ptr<VesselNetwork<3> >)) &Owen11CellPopulationGenerator3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetGridCalculator", 
            (void(Owen11CellPopulationGenerator3::*)(::std::shared_ptr<GridCalculator<3> >)) &Owen11CellPopulationGenerator3::SetGridCalculator, 
            " " , py::arg("pGrid") )
        .def(
            "SetReferenceLengthScale", 
            (void(Owen11CellPopulationGenerator3::*)(::QLength)) &Owen11CellPopulationGenerator3::SetReferenceLengthScale, 
            " " , py::arg("lengthScale") )
        .def(
            "SetTumourRadius", 
            (void(Owen11CellPopulationGenerator3::*)(::QLength)) &Owen11CellPopulationGenerator3::SetTumourRadius, 
            " " , py::arg("tumourRadius") )
        .def(
            "Update", 
            (::std::shared_ptr<CaBasedCellPopulation<3> >(Owen11CellPopulationGenerator3::*)()) &Owen11CellPopulationGenerator3::Update, 
            " "  )
    ;
}
