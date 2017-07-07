#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "CellPopulationActorGenerator.hpp"

#include "CellPopulationActorGenerator3.cppwg.hpp"

namespace py = pybind11;
typedef CellPopulationActorGenerator<3 > CellPopulationActorGenerator3;
;

class CellPopulationActorGenerator3_Overloads : public CellPopulationActorGenerator3{
    public:
    using CellPopulationActorGenerator3::CellPopulationActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            CellPopulationActorGenerator3,
            AddActor,
            pRenderer);
    }

};
void register_CellPopulationActorGenerator3_class(py::module &m){
py::class_<CellPopulationActorGenerator3 , CellPopulationActorGenerator3_Overloads   >(m, "CellPopulationActorGenerator3")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(CellPopulationActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator3::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "AddMeshBasedCellPopulationActor", 
            (void(CellPopulationActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator3::AddMeshBasedCellPopulationActor, 
            " " , py::arg("pRenderer") )
        .def(
            "AddVertexBasedCellPopulationActor", 
            (void(CellPopulationActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator3::AddVertexBasedCellPopulationActor, 
            " " , py::arg("pRenderer") )
        .def(
            "AddCaBasedCellPopulationActor", 
            (void(CellPopulationActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator3::AddCaBasedCellPopulationActor, 
            " " , py::arg("pRenderer") )
        .def(
            "AddPottsBasedCellPopulationActor", 
            (void(CellPopulationActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator3::AddPottsBasedCellPopulationActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetCellPopulation", 
            (void(CellPopulationActorGenerator3::*)(::std::shared_ptr<AbstractCellPopulation<3, 3> >)) &CellPopulationActorGenerator3::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetShowVoronoiMeshEdges", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetShowVoronoiMeshEdges, 
            " " , py::arg("showEdges") )
        .def(
            "SetShowMutableMeshEdges", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetShowMutableMeshEdges, 
            " " , py::arg("showEdges") )
        .def(
            "SetShowPottsMeshEdges", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetShowPottsMeshEdges, 
            " " , py::arg("showEdges") )
        .def(
            "SetShowPottsMeshOutlines", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetShowPottsMeshOutlines, 
            " " , py::arg("showOutlines") )
        .def(
            "SetColorByCellType", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetColorByCellType, 
            " " , py::arg("colorByCellType") )
        .def(
            "SetColorByCellMutationState", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetColorByCellMutationState, 
            " " , py::arg("colorByCellMutationState") )
        .def(
            "SetColorByCellLabel", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetColorByCellLabel, 
            " " , py::arg("colorByCellLabel") )
        .def(
            "SetColorByUserDefined", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetColorByUserDefined, 
            " " , py::arg("colorByCellUserDefined") )
        .def(
            "SetColorByCellData", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetColorByCellData, 
            " " , py::arg("colorByCellData") )
        .def(
            "SetShowCellCentres", 
            (void(CellPopulationActorGenerator3::*)(bool)) &CellPopulationActorGenerator3::SetShowCellCentres, 
            " " , py::arg("showCentres") )
    ;
}
