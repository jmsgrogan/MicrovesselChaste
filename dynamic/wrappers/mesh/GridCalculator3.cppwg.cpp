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

#include "GridCalculator3.cppwg.hpp"

namespace py = pybind11;
typedef GridCalculator<3 > GridCalculator3;
;

void register_GridCalculator3_class(py::module &m){
py::class_<GridCalculator3    >(m, "GridCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<GridCalculator<3> >(*)()) &GridCalculator3::Create, 
            " " )
        .def(
            "CellPopulationIsSet", 
            (bool(GridCalculator3::*)()) &GridCalculator3::CellPopulationIsSet, 
            " " )
        .def(
            "GetPointMap", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(GridCalculator3::*)(::std::vector<DimensionalChastePoint<3>, std::allocator<DimensionalChastePoint<3> > > const &)) &GridCalculator3::GetPointMap, 
            " " , py::arg("rInputPoints"))
        .def(
            "GetPointMap", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(GridCalculator3::*)(::vtkSmartPointer<vtkPoints>)) &GridCalculator3::GetPointMap, 
            " " , py::arg("pInputPoints"))
        .def(
            "rGetCellMap", 
            (::std::vector<std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > >, std::allocator<std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > > > > const &(GridCalculator3::*)(bool)) &GridCalculator3::rGetCellMap, 
            " " , py::arg("update") = true)
        .def(
            "GetVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<3> >(GridCalculator3::*)()) &GridCalculator3::GetVesselNetwork, 
            " " )
        .def(
            "rGetVesselNodeMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >, std::allocator<std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > > > const &(GridCalculator3::*)(bool)) &GridCalculator3::rGetVesselNodeMap, 
            " " , py::arg("update") = true)
        .def(
            "rGetSegmentMap", 
            (::std::vector<std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >, std::allocator<std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > > > > const &(GridCalculator3::*)(bool, bool)) &GridCalculator3::rGetSegmentMap, 
            " " , py::arg("update") = true, py::arg("useVesselSurface") = false)
        .def(
            "GetGrid", 
            (::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >(GridCalculator3::*)()) &GridCalculator3::GetGrid, 
            " " )
        .def(
            "HasStructuredGrid", 
            (bool(GridCalculator3::*)()) &GridCalculator3::HasStructuredGrid, 
            " " )
        .def(
            "HasUnstructuredGrid", 
            (bool(GridCalculator3::*)()) &GridCalculator3::HasUnstructuredGrid, 
            " " )
        .def(
            "IsSegmentAtLocation", 
            (bool(GridCalculator3::*)(unsigned int, bool)) &GridCalculator3::IsSegmentAtLocation, 
            " " , py::arg("index"), py::arg("update"))
        .def(
            "SetCellPopulation", 
            (void(GridCalculator3::*)(::AbstractCellPopulation<3, 3> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &GridCalculator3::SetCellPopulation, 
            " " , py::arg("rCellPopulation"), py::arg("cellPopulationReferenceLength"), py::arg("cellPopulationReferenceConcentration"))
        .def(
            "SetVesselNetwork", 
            (void(GridCalculator3::*)(::std::shared_ptr<VesselNetwork<3> >)) &GridCalculator3::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetGrid", 
            (void(GridCalculator3::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >)) &GridCalculator3::SetGrid, 
            " " , py::arg("pGrid"))
    ;
}
