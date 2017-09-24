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
#include "GridCalculator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef GridCalculator<2 > GridCalculator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_GridCalculator2_class(py::module &m){
py::class_<GridCalculator2  , std::shared_ptr<GridCalculator2 >   >(m, "GridCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<GridCalculator<2> >(*)()) &GridCalculator2::Create, 
            " "  )
        .def(
            "CellPopulationIsSet", 
            (bool(GridCalculator2::*)()) &GridCalculator2::CellPopulationIsSet, 
            " "  )
        .def(
            "GetPointMap", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(GridCalculator2::*)(::std::vector<Vertex<2>, std::allocator<Vertex<2> > > const &)) &GridCalculator2::GetPointMap, 
            " " , py::arg("rInputPoints") )
        .def(
            "GetPointMap", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(GridCalculator2::*)(::vtkSmartPointer<vtkPoints>)) &GridCalculator2::GetPointMap, 
            " " , py::arg("pInputPoints") )
        .def(
            "rGetCellMap", 
            (::std::vector<std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > >, std::allocator<std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > > > > const &(GridCalculator2::*)(bool)) &GridCalculator2::rGetCellMap, 
            " " , py::arg("update") = true , py::return_value_policy::reference_internal)
        .def(
            "GetVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<2> >(GridCalculator2::*)()) &GridCalculator2::GetVesselNetwork, 
            " "  )
        .def(
            "rGetVesselNodeMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >, std::allocator<std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > > > const &(GridCalculator2::*)(bool)) &GridCalculator2::rGetVesselNodeMap, 
            " " , py::arg("update") = true , py::return_value_policy::reference_internal)
        .def_static(
            "GetVesselNodeMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >, std::allocator<std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > > >(*)(::vtkSmartPointer<vtkUnstructuredGrid>, ::std::shared_ptr<VesselNetwork<2> >, ::QLength)) &GridCalculator2::GetVesselNodeMap, 
            " " , py::arg("pGrid"), py::arg("pNetwork"), py::arg("referenceLength") )
        .def(
            "rGetSegmentMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >, std::allocator<std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > > > > const &(GridCalculator2::*)(bool, bool)) &GridCalculator2::rGetSegmentMap, 
            " " , py::arg("update") = true, py::arg("useVesselSurface") = false , py::return_value_policy::reference_internal)
        .def_static(
            "GetSegmentMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >, std::allocator<std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > > > >(*)(::vtkSmartPointer<vtkUnstructuredGrid>, ::std::shared_ptr<VesselNetwork<2> >, ::QLength)) &GridCalculator2::GetSegmentMap, 
            " " , py::arg("pGrid"), py::arg("pNetwork"), py::arg("referenceLength") )
        .def(
            "GetGrid", 
            (::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >(GridCalculator2::*)()) &GridCalculator2::GetGrid, 
            " "  )
        .def(
            "HasStructuredGrid", 
            (bool(GridCalculator2::*)()) &GridCalculator2::HasStructuredGrid, 
            " "  )
        .def(
            "HasUnstructuredGrid", 
            (bool(GridCalculator2::*)()) &GridCalculator2::HasUnstructuredGrid, 
            " "  )
        .def(
            "IsSegmentAtLocation", 
            (bool(GridCalculator2::*)(unsigned int, bool)) &GridCalculator2::IsSegmentAtLocation, 
            " " , py::arg("index"), py::arg("update") )
        .def(
            "SetCellPopulation", 
            (void(GridCalculator2::*)(::AbstractCellPopulation<2, 2> &, ::QLength, ::QConcentration)) &GridCalculator2::SetCellPopulation, 
            " " , py::arg("rCellPopulation"), py::arg("cellPopulationReferenceLength"), py::arg("cellPopulationReferenceConcentration") )
        .def(
            "SetVesselNetwork", 
            (void(GridCalculator2::*)(::std::shared_ptr<VesselNetwork<2> >)) &GridCalculator2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetGrid", 
            (void(GridCalculator2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &GridCalculator2::SetGrid, 
            " " , py::arg("pGrid") )
    ;
}
