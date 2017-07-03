#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "GridCalculator.hpp"

#include "GridCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef GridCalculator<2 > GridCalculator2;
;

void register_GridCalculator2_class(py::module &m){
py::class_<GridCalculator2    >(m, "GridCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<GridCalculator<2> >(*)()) &GridCalculator2::Create, 
            " " )
        .def(
            "CellPopulationIsSet", 
            (bool(GridCalculator2::*)()) &GridCalculator2::CellPopulationIsSet, 
            " " )
        .def(
            "GetPointMap", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(GridCalculator2::*)(::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > > const &)) &GridCalculator2::GetPointMap, 
            " " , py::arg("rInputPoints"))
        .def(
            "GetPointMap", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(GridCalculator2::*)(::vtkSmartPointer<vtkPoints>)) &GridCalculator2::GetPointMap, 
            " " , py::arg("pInputPoints"))
        .def(
            "rGetCellMap", 
            (::std::vector<std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > >, std::allocator<std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > > > > const &(GridCalculator2::*)(bool)) &GridCalculator2::rGetCellMap, 
            " " , py::arg("update") = true)
        .def(
            "GetVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<2> >(GridCalculator2::*)()) &GridCalculator2::GetVesselNetwork, 
            " " )
        .def(
            "rGetVesselNodeMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >, std::allocator<std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > > > const &(GridCalculator2::*)(bool)) &GridCalculator2::rGetVesselNodeMap, 
            " " , py::arg("update") = true)
        .def(
            "rGetSegmentMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >, std::allocator<std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > > > > const &(GridCalculator2::*)(bool, bool)) &GridCalculator2::rGetSegmentMap, 
            " " , py::arg("update") = true, py::arg("useVesselSurface") = false)
        .def(
            "GetGrid", 
            (::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >(GridCalculator2::*)()) &GridCalculator2::GetGrid, 
            " " )
        .def(
            "HasStructuredGrid", 
            (bool(GridCalculator2::*)()) &GridCalculator2::HasStructuredGrid, 
            " " )
        .def(
            "HasUnstructuredGrid", 
            (bool(GridCalculator2::*)()) &GridCalculator2::HasUnstructuredGrid, 
            " " )
        .def(
            "IsSegmentAtLocation", 
            (bool(GridCalculator2::*)(unsigned int, bool)) &GridCalculator2::IsSegmentAtLocation, 
            " " , py::arg("index"), py::arg("update"))
        .def(
            "SetCellPopulation", 
            (void(GridCalculator2::*)(::AbstractCellPopulation<2, 2> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &GridCalculator2::SetCellPopulation, 
            " " , py::arg("rCellPopulation"), py::arg("cellPopulationReferenceLength"), py::arg("cellPopulationReferenceConcentration"))
        .def(
            "SetVesselNetwork", 
            (void(GridCalculator2::*)(::std::shared_ptr<VesselNetwork<2> >)) &GridCalculator2::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetGrid", 
            (void(GridCalculator2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &GridCalculator2::SetGrid, 
            " " , py::arg("pGrid"))
    ;
}
