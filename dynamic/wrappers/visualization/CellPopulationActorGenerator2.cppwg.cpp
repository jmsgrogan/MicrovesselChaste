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
#include "CellPopulationActorGenerator.hpp"

#include "PythonObjectConverters.hpp"
#include "CellPopulationActorGenerator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef CellPopulationActorGenerator<2 > CellPopulationActorGenerator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class CellPopulationActorGenerator2_Overloads : public CellPopulationActorGenerator2{
    public:
    using CellPopulationActorGenerator2::CellPopulationActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            CellPopulationActorGenerator2,
            AddActor,
            pRenderer);
    }

};
void register_CellPopulationActorGenerator2_class(py::module &m){
py::class_<CellPopulationActorGenerator2 , CellPopulationActorGenerator2_Overloads , std::shared_ptr<CellPopulationActorGenerator2 >  , AbstractActorGenerator<2>  >(m, "CellPopulationActorGenerator2")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(CellPopulationActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator2::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "AddMeshBasedCellPopulationActor", 
            (void(CellPopulationActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator2::AddMeshBasedCellPopulationActor, 
            " " , py::arg("pRenderer") )
        .def(
            "AddVertexBasedCellPopulationActor", 
            (void(CellPopulationActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator2::AddVertexBasedCellPopulationActor, 
            " " , py::arg("pRenderer") )
        .def(
            "AddCaBasedCellPopulationActor", 
            (void(CellPopulationActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator2::AddCaBasedCellPopulationActor, 
            " " , py::arg("pRenderer") )
        .def(
            "AddPottsBasedCellPopulationActor", 
            (void(CellPopulationActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &CellPopulationActorGenerator2::AddPottsBasedCellPopulationActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetCellPopulation", 
            (void(CellPopulationActorGenerator2::*)(::std::shared_ptr<AbstractCellPopulation<2, 2> >)) &CellPopulationActorGenerator2::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetShowVoronoiMeshEdges", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetShowVoronoiMeshEdges, 
            " " , py::arg("showEdges") )
        .def(
            "SetShowMutableMeshEdges", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetShowMutableMeshEdges, 
            " " , py::arg("showEdges") )
        .def(
            "SetShowPottsMeshEdges", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetShowPottsMeshEdges, 
            " " , py::arg("showEdges") )
        .def(
            "SetShowPottsMeshOutlines", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetShowPottsMeshOutlines, 
            " " , py::arg("showOutlines") )
        .def(
            "SetColorByCellType", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetColorByCellType, 
            " " , py::arg("colorByCellType") )
        .def(
            "SetColorByCellMutationState", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetColorByCellMutationState, 
            " " , py::arg("colorByCellMutationState") )
        .def(
            "SetColorByCellLabel", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetColorByCellLabel, 
            " " , py::arg("colorByCellLabel") )
        .def(
            "SetColorByUserDefined", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetColorByUserDefined, 
            " " , py::arg("colorByCellUserDefined") )
        .def(
            "SetColorByCellData", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetColorByCellData, 
            " " , py::arg("colorByCellData") )
        .def(
            "SetShowCellCentres", 
            (void(CellPopulationActorGenerator2::*)(bool)) &CellPopulationActorGenerator2::SetShowCellCentres, 
            " " , py::arg("showCentres") )
    ;
}
