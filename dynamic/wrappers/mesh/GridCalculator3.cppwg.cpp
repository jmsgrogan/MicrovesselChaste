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
#include "GridCalculator.hpp"

#include "PythonObjectConverters.hpp"
#include "GridCalculator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef GridCalculator<3 > GridCalculator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_GridCalculator3_class(py::module &m){
py::class_<GridCalculator3  , std::shared_ptr<GridCalculator3 >   >(m, "GridCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<GridCalculator<3> >(*)()) &GridCalculator3::Create, 
            " "  )
        .def(
            "CellPopulationIsSet", 
            (bool(GridCalculator3::*)()) &GridCalculator3::CellPopulationIsSet, 
            " "  )
        .def(
            "GetPointMap", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(GridCalculator3::*)(::std::vector<Vertex<3>, std::allocator<Vertex<3> > > const &)) &GridCalculator3::GetPointMap, 
            " " , py::arg("rInputPoints") )
        .def(
            "GetPointMap", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(GridCalculator3::*)(::vtkSmartPointer<vtkPoints>)) &GridCalculator3::GetPointMap, 
            " " , py::arg("pInputPoints") )
        .def(
            "rGetCellMap", 
            (::std::vector<std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > >, std::allocator<std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > > > > const &(GridCalculator3::*)(bool)) &GridCalculator3::rGetCellMap, 
            " " , py::arg("update") = true , py::return_value_policy::reference_internal)
        .def(
            "GetVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<3> >(GridCalculator3::*)()) &GridCalculator3::GetVesselNetwork, 
            " "  )
        .def(
            "rGetVesselNodeMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >, std::allocator<std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > > > const &(GridCalculator3::*)(bool)) &GridCalculator3::rGetVesselNodeMap, 
            " " , py::arg("update") = true , py::return_value_policy::reference_internal)
        .def_static(
            "GetVesselNodeMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >, std::allocator<std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > > >(*)(::vtkSmartPointer<vtkUnstructuredGrid>, ::std::shared_ptr<VesselNetwork<3> >, ::QLength)) &GridCalculator3::GetVesselNodeMap, 
            " " , py::arg("pGrid"), py::arg("pNetwork"), py::arg("referenceLength") )
        .def(
            "rGetSegmentMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >, std::allocator<std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > > > > const &(GridCalculator3::*)(bool, bool)) &GridCalculator3::rGetSegmentMap, 
            " " , py::arg("update") = true, py::arg("useVesselSurface") = false , py::return_value_policy::reference_internal)
        .def_static(
            "GetSegmentMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >, std::allocator<std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > > > >(*)(::vtkSmartPointer<vtkUnstructuredGrid>, ::std::shared_ptr<VesselNetwork<3> >, ::QLength)) &GridCalculator3::GetSegmentMap, 
            " " , py::arg("pGrid"), py::arg("pNetwork"), py::arg("referenceLength") )
        .def(
            "GetGrid", 
            (::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >(GridCalculator3::*)()) &GridCalculator3::GetGrid, 
            " "  )
        .def(
            "HasStructuredGrid", 
            (bool(GridCalculator3::*)()) &GridCalculator3::HasStructuredGrid, 
            " "  )
        .def(
            "HasUnstructuredGrid", 
            (bool(GridCalculator3::*)()) &GridCalculator3::HasUnstructuredGrid, 
            " "  )
        .def(
            "IsSegmentAtLocation", 
            (bool(GridCalculator3::*)(unsigned int, bool)) &GridCalculator3::IsSegmentAtLocation, 
            " " , py::arg("index"), py::arg("update") )
        .def(
            "SetCellPopulation", 
            (void(GridCalculator3::*)(::AbstractCellPopulation<3, 3> &, ::QLength, ::QConcentration)) &GridCalculator3::SetCellPopulation, 
            " " , py::arg("rCellPopulation"), py::arg("cellPopulationReferenceLength"), py::arg("cellPopulationReferenceConcentration") )
        .def(
            "SetVesselNetwork", 
            (void(GridCalculator3::*)(::std::shared_ptr<VesselNetwork<3> >)) &GridCalculator3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetGrid", 
            (void(GridCalculator3::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >)) &GridCalculator3::SetGrid, 
            " " , py::arg("pGrid") )
    ;
}
