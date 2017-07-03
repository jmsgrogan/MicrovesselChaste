#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Part.hpp"

#include "Part2.cppwg.hpp"

namespace py = pybind11;
typedef Part<2 > Part2;
;

void register_Part2_class(py::module &m){
py::class_<Part2    >(m, "Part2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<Part<2> >(*)()) &Part2::Create, 
            " " )
        .def(
            "AddAttribute", 
            (void(Part2::*)(::std::string const &, double)) &Part2::AddAttribute, 
            " " , py::arg("rLabel"), py::arg("value"))
        .def(
            "AddAttributeToEdgeIfFound", 
            (void(Part2::*)(::DimensionalChastePoint<2>, ::std::string const &, double)) &Part2::AddAttributeToEdgeIfFound, 
            " " , py::arg("loc"), py::arg("rLabel"), py::arg("value"))
        .def(
            "AddAttributeToPolygons", 
            (void(Part2::*)(::std::string const &, double)) &Part2::AddAttributeToPolygons, 
            " " , py::arg("rLabel"), py::arg("value"))
        .def(
            "AddAttributeToPolygonIfFound", 
            (void(Part2::*)(::DimensionalChastePoint<2>, ::std::string const &, double)) &Part2::AddAttributeToPolygonIfFound, 
            " " , py::arg("loc"), py::arg("rLabel"), py::arg("value"))
        .def(
            "AddCircle", 
            (::std::shared_ptr<Polygon<2> >(Part2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::DimensionalChastePoint<2>, unsigned int)) &Part2::AddCircle, 
            " " , py::arg("radius"), py::arg("centre"), py::arg("numSegments") = 24)
        .def(
            "AddCylinder", 
            (void(Part2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::DimensionalChastePoint<2>, unsigned int)) &Part2::AddCylinder, 
            " " , py::arg("radius"), py::arg("depth"), py::arg("centre"), py::arg("numSegments") = 24)
        .def(
            "AddCuboid", 
            (void(Part2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::DimensionalChastePoint<2>)) &Part2::AddCuboid, 
            " " , py::arg("sizeX"), py::arg("sizeY"), py::arg("sizeZ"), py::arg("origin"))
        .def(
            "AddHoleMarker", 
            (void(Part2::*)(::DimensionalChastePoint<2>)) &Part2::AddHoleMarker, 
            " " , py::arg("location"))
        .def(
            "AddRegionMarker", 
            (void(Part2::*)(::DimensionalChastePoint<2>, unsigned int)) &Part2::AddRegionMarker, 
            " " , py::arg("location"), py::arg("value"))
        .def(
            "AddPolygon", 
            (::std::shared_ptr<Polygon<2> >(Part2::*)(::std::vector<std::shared_ptr<DimensionalChastePoint<2> >, std::allocator<std::shared_ptr<DimensionalChastePoint<2> > > >, bool, ::std::shared_ptr<Facet<2> >)) &Part2::AddPolygon, 
            " " , py::arg("vertices"), py::arg("newFacet") = false, py::arg("pFacet") = std::shared_ptr<Facet<DIM> >())
        .def(
            "AddPolygon", 
            (::std::shared_ptr<Polygon<2> >(Part2::*)(::std::shared_ptr<Polygon<2> >, bool, ::std::shared_ptr<Facet<2> >)) &Part2::AddPolygon, 
            " " , py::arg("pPolygon"), py::arg("newFacet") = false, py::arg("pFacet") = std::shared_ptr<Facet<DIM> >())
        .def(
            "AddRectangle", 
            (::std::shared_ptr<Polygon<2> >(Part2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::DimensionalChastePoint<2>)) &Part2::AddRectangle, 
            " " , py::arg("sizeX"), py::arg("sizeY"), py::arg("origin"))
        .def(
            "AddVesselNetwork", 
            (void(Part2::*)(::std::shared_ptr<VesselNetwork<2> >, bool, bool)) &Part2::AddVesselNetwork, 
            " " , py::arg("pVesselNetwork"), py::arg("surface") = false, py::arg("removeVesselRegion") = true)
        .def(
            "AppendPart", 
            (void(Part2::*)(::std::shared_ptr<Part<2> >)) &Part2::AppendPart, 
            " " , py::arg("pPart"))
        .def(
            "BooleanWithNetwork", 
            (void(Part2::*)(::std::shared_ptr<VesselNetwork<2> >)) &Part2::BooleanWithNetwork, 
            " " , py::arg("pVesselNetwork"))
        .def(
            "Extrude", 
            (void(Part2::*)(::std::shared_ptr<Polygon<2> >, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &Part2::Extrude, 
            " " , py::arg("pPolygon"), py::arg("distance"))
        .def(
            "GetBoundingBox", 
            (::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > >(Part2::*)()) &Part2::GetBoundingBox, 
            " " )
        .def(
            "GetContainingGridIndices", 
            (::std::vector<unsigned int, std::allocator<unsigned int> >(Part2::*)(unsigned int, unsigned int, unsigned int, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &Part2::GetContainingGridIndices, 
            " " , py::arg("num_x"), py::arg("num_y"), py::arg("num_z"), py::arg("spacing"))
        .def(
            "GetHoleMarkers", 
            (::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >(Part2::*)()) &Part2::GetHoleMarkers, 
            " " )
        .def(
            "GetRegionMarkers", 
            (::std::vector<std::pair<DimensionalChastePoint<2>, unsigned int>, std::allocator<std::pair<DimensionalChastePoint<2>, unsigned int> > >(Part2::*)()) &Part2::GetRegionMarkers, 
            " " )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Part2::*)()) &Part2::GetAttributes, 
            " " )
        .def(
            "GetAttributesKeysForMesh", 
            (::std::map<unsigned int, std::basic_string<char>, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, std::basic_string<char> > > >(Part2::*)(bool)) &Part2::GetAttributesKeysForMesh, 
            " " , py::arg("update") = true)
        .def(
            "GetFacets", 
            (::std::vector<std::shared_ptr<Facet<2> >, std::allocator<std::shared_ptr<Facet<2> > > >(Part2::*)()) &Part2::GetFacets, 
            " " )
        .def(
            "GetFacet", 
            (::std::shared_ptr<Facet<2> >(Part2::*)(::DimensionalChastePoint<2> const &)) &Part2::GetFacet, 
            " " , py::arg("rLocation"))
        .def(
            "GetPolygons", 
            (::std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > >(Part2::*)()) &Part2::GetPolygons, 
            " " )
        .def(
            "GetReferenceLengthScale", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(Part2::*)()) &Part2::GetReferenceLengthScale, 
            " " )
        .def(
            "GetSegmentIndices", 
            (::std::vector<std::pair<std::pair<unsigned int, unsigned int>, unsigned int>, std::allocator<std::pair<std::pair<unsigned int, unsigned int>, unsigned int> > >(Part2::*)()) &Part2::GetSegmentIndices, 
            " " )
        .def(
            "GetVertices", 
            (::std::vector<std::shared_ptr<DimensionalChastePoint<2> >, std::allocator<std::shared_ptr<DimensionalChastePoint<2> > > >(Part2::*)()) &Part2::GetVertices, 
            " " )
        .def(
            "GetVertexLocations", 
            (::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >(Part2::*)()) &Part2::GetVertexLocations, 
            " " )
        .def(
            "GetVtk", 
            (::vtkSmartPointer<vtkPolyData>(Part2::*)(bool)) &Part2::GetVtk, 
            " " , py::arg("includeEdges") = false)
        .def(
            "GetKeyForAttributes", 
            (unsigned int(Part2::*)(::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >)) &Part2::GetKeyForAttributes, 
            " " , py::arg("rAttributes"))
        .def(
            "EdgeHasAttribute", 
            (bool(Part2::*)(::DimensionalChastePoint<2>, ::std::string const &)) &Part2::EdgeHasAttribute, 
            " " , py::arg("loc"), py::arg("rLabel"))
        .def(
            "IsPointInPart", 
            (bool(Part2::*)(::DimensionalChastePoint<2>)) &Part2::IsPointInPart, 
            " " , py::arg("location"))
        .def(
            "IsPointInPart", 
            (::std::vector<bool, std::allocator<bool> >(Part2::*)(::vtkSmartPointer<vtkPoints>)) &Part2::IsPointInPart, 
            " " , py::arg("pPoints"))
        .def(
            "MergeCoincidentVertices", 
            (void(Part2::*)()) &Part2::MergeCoincidentVertices, 
            " " )
        .def(
            "RotateAboutAxis", 
            (void(Part2::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Part2::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle"))
        .def(
            "SetReferenceLengthScale", 
            (void(Part2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &Part2::SetReferenceLengthScale, 
            " " , py::arg("referenceLength"))
        .def(
            "Translate", 
            (void(Part2::*)(::DimensionalChastePoint<2>)) &Part2::Translate, 
            " " , py::arg("vector"))
        .def(
            "Write", 
            (void(Part2::*)(::std::string const &, ::GeometryFormat::Value, bool)) &Part2::Write, 
            " " , py::arg("rFilename"), py::arg("format") = ::GeometryFormat::Value::VTP, py::arg("includeEdges") = false)
    ;
}
