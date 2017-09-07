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
#include "Owen11CaBasedDivisionRule.hpp"

#include "PythonObjectConverters.hpp"
#include "Owen11CaBasedDivisionRule3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef Owen11CaBasedDivisionRule<3 > Owen11CaBasedDivisionRule3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef unsigned int unsignedint;

class Owen11CaBasedDivisionRule3_Overloads : public Owen11CaBasedDivisionRule3{
    public:
    using Owen11CaBasedDivisionRule3::Owen11CaBasedDivisionRule;
    bool IsRoomToDivide(::CellPtr pParentCell, ::CaBasedCellPopulation<3> & rCellPopulation) override {
        PYBIND11_OVERLOAD(
            bool,
            Owen11CaBasedDivisionRule3,
            IsRoomToDivide,
            pParentCell, 
rCellPopulation);
    }
    unsigned int CalculateDaughterNodeIndex(::CellPtr pNewCell, ::CellPtr pParentCell, ::CaBasedCellPopulation<3> & rCellPopulation) override {
        PYBIND11_OVERLOAD(
            unsignedint,
            Owen11CaBasedDivisionRule3,
            CalculateDaughterNodeIndex,
            pNewCell, 
pParentCell, 
rCellPopulation);
    }

};
void register_Owen11CaBasedDivisionRule3_class(py::module &m){
py::class_<Owen11CaBasedDivisionRule3 , Owen11CaBasedDivisionRule3_Overloads , std::shared_ptr<Owen11CaBasedDivisionRule3 >   >(m, "Owen11CaBasedDivisionRule3")
        .def(py::init< >())
        .def(
            "IsRoomToDivide", 
            (bool(Owen11CaBasedDivisionRule3::*)(::CellPtr, ::CaBasedCellPopulation<3> &)) &Owen11CaBasedDivisionRule3::IsRoomToDivide, 
            " " , py::arg("pParentCell"), py::arg("rCellPopulation") )
        .def(
            "SetVesselNetwork", 
            (void(Owen11CaBasedDivisionRule3::*)(::std::shared_ptr<VesselNetwork<3> >)) &Owen11CaBasedDivisionRule3::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "SetGridCalculator", 
            (void(Owen11CaBasedDivisionRule3::*)(::std::shared_ptr<GridCalculator<3> >)) &Owen11CaBasedDivisionRule3::SetGridCalculator, 
            " " , py::arg("mpGridCalculator") )
        .def(
            "SetReferenceLengthScale", 
            (void(Owen11CaBasedDivisionRule3::*)(::QLength)) &Owen11CaBasedDivisionRule3::SetReferenceLengthScale, 
            " " , py::arg("referenceLengthScale") )
        .def(
            "CalculateDaughterNodeIndex", 
            (unsigned int(Owen11CaBasedDivisionRule3::*)(::CellPtr, ::CellPtr, ::CaBasedCellPopulation<3> &)) &Owen11CaBasedDivisionRule3::CalculateDaughterNodeIndex, 
            " " , py::arg("pNewCell"), py::arg("pParentCell"), py::arg("rCellPopulation") )
    ;
}
