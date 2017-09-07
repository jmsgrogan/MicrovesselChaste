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
#include "Part.hpp"

#include "PythonObjectConverters.hpp"
#include "Part2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef Part<2 > Part2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_Part2_class(py::module &m){
py::class_<Part2  , std::shared_ptr<Part2 >   >(m, "Part2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<Part<2> >(*)()) &Part2::Create, 
            " "  )
        .def(
            "AddAttribute", 
            (void(Part2::*)(::std::string const &, double)) &Part2::AddAttribute, 
            " " , py::arg("rLabel"), py::arg("value") )
        .def(
            "AddAttributeToEdgeIfFound", 
            (void(Part2::*)(::Vertex<2> const &, ::std::string const &, double)) &Part2::AddAttributeToEdgeIfFound, 
            " " , py::arg("rLoc"), py::arg("rLabel"), py::arg("value") )
        .def(
            "AddAttributeToPolygons", 
            (void(Part2::*)(::std::string const &, double)) &Part2::AddAttributeToPolygons, 
            " " , py::arg("rLabel"), py::arg("value") )
        .def(
            "AddAttributeToPolygonIfFound", 
            (void(Part2::*)(::Vertex<2> const &, ::std::string const &, double)) &Part2::AddAttributeToPolygonIfFound, 
            " " , py::arg("rLoc"), py::arg("rLabel"), py::arg("value") )
        .def(
            "AddCircle", 
            (::std::shared_ptr<Polygon<2> >(Part2::*)(::QLength, ::Vertex<2>, unsigned int)) &Part2::AddCircle, 
            " " , py::arg("radius"), py::arg("centre") = Vertex<2>(), py::arg("numSegments") = 24 )
        .def(
            "AddCylinder", 
            (void(Part2::*)(::QLength, ::QLength, ::Vertex<2>, unsigned int)) &Part2::AddCylinder, 
            " " , py::arg("radius"), py::arg("depth"), py::arg("centre") = Vertex<2>(), py::arg("numSegments") = 24 )
        .def(
            "AddCuboid", 
            (void(Part2::*)(::QLength, ::QLength, ::QLength, ::Vertex<2>)) &Part2::AddCuboid, 
            " " , py::arg("sizeX"), py::arg("sizeY"), py::arg("sizeZ"), py::arg("origin") = Vertex<2>() )
        .def(
            "AddHoleMarker", 
            (void(Part2::*)(::Vertex<2>)) &Part2::AddHoleMarker, 
            " " , py::arg("location") )
        .def(
            "AddRegionMarker", 
            (void(Part2::*)(::Vertex<2>, unsigned int)) &Part2::AddRegionMarker, 
            " " , py::arg("location"), py::arg("value") )
        .def(
            "AddPolygon", 
            (::std::shared_ptr<Polygon<2> >(Part2::*)(::std::vector<std::shared_ptr<Vertex<2> >, std::allocator<std::shared_ptr<Vertex<2> > > > const &, bool, ::std::shared_ptr<Facet<2> >)) &Part2::AddPolygon, 
            " " , py::arg("vertices"), py::arg("newFacet") = false, py::arg("pFacet") = FacetPtr<2>() )
        .def(
            "AddPolygon", 
            (::std::shared_ptr<Polygon<2> >(Part2::*)(::std::shared_ptr<Polygon<2> >, bool, ::std::shared_ptr<Facet<2> >)) &Part2::AddPolygon, 
            " " , py::arg("pPolygon"), py::arg("newFacet") = false, py::arg("pFacet") = FacetPtr<2>() )
        .def(
            "AddRectangle", 
            (::std::shared_ptr<Polygon<2> >(Part2::*)(::QLength, ::QLength, ::Vertex<2>)) &Part2::AddRectangle, 
            " " , py::arg("sizeX"), py::arg("sizeY"), py::arg("origin") = Vertex<2>() )
        .def(
            "AddVesselNetwork", 
            (void(Part2::*)(::std::shared_ptr<VesselNetwork<2> >, bool, bool)) &Part2::AddVesselNetwork, 
            " " , py::arg("pVesselNetwork"), py::arg("surface") = false, py::arg("removeVesselRegion") = true )
        .def(
            "AppendPart", 
            (void(Part2::*)(::std::shared_ptr<Part<2> >)) &Part2::AppendPart, 
            " " , py::arg("pPart") )
        .def(
            "BooleanWithNetwork", 
            (void(Part2::*)(::std::shared_ptr<VesselNetwork<2> >)) &Part2::BooleanWithNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "Extrude", 
            (void(Part2::*)(::std::shared_ptr<Polygon<2> >, ::QLength)) &Part2::Extrude, 
            " " , py::arg("pPolygon"), py::arg("distance") )
        .def(
            "GetBoundingBox", 
            (::std::array<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 6>(Part2::*)()) &Part2::GetBoundingBox, 
            " "  )
        .def(
            "GetContainingGridIndices", 
            (::std::vector<unsigned int, std::allocator<unsigned int> >(Part2::*)(unsigned int, unsigned int, unsigned int, ::QLength)) &Part2::GetContainingGridIndices, 
            " " , py::arg("num_x"), py::arg("num_y"), py::arg("num_z"), py::arg("spacing") )
        .def(
            "GetHoleMarkers", 
            (::std::vector<Vertex<2>, std::allocator<Vertex<2> > >(Part2::*)()) &Part2::GetHoleMarkers, 
            " "  )
        .def(
            "GetRegionMarkers", 
            (::std::vector<std::pair<Vertex<2>, unsigned int>, std::allocator<std::pair<Vertex<2>, unsigned int> > >(Part2::*)()) &Part2::GetRegionMarkers, 
            " "  )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Part2::*)()) &Part2::GetAttributes, 
            " "  )
        .def(
            "GetAttributesKeysForMesh", 
            (::std::map<unsigned int, std::basic_string<char>, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, std::basic_string<char> > > >(Part2::*)(bool)) &Part2::GetAttributesKeysForMesh, 
            " " , py::arg("update") = true )
        .def(
            "GetFacets", 
            (::std::vector<std::shared_ptr<Facet<2> >, std::allocator<std::shared_ptr<Facet<2> > > >(Part2::*)()) &Part2::GetFacets, 
            " "  )
        .def(
            "GetFacet", 
            (::std::shared_ptr<Facet<2> >(Part2::*)(::Vertex<2> const &)) &Part2::GetFacet, 
            " " , py::arg("rLocation") )
        .def(
            "GetPolygons", 
            (::std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > >(Part2::*)()) &Part2::GetPolygons, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(Part2::*)()) &Part2::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetSegmentIndices", 
            (::std::vector<std::pair<std::pair<unsigned int, unsigned int>, unsigned int>, std::allocator<std::pair<std::pair<unsigned int, unsigned int>, unsigned int> > >(Part2::*)()) &Part2::GetSegmentIndices, 
            " "  )
        .def(
            "GetVertices", 
            (::std::vector<std::shared_ptr<Vertex<2> >, std::allocator<std::shared_ptr<Vertex<2> > > >(Part2::*)()) &Part2::GetVertices, 
            " "  )
        .def(
            "GetVtk", 
            (::vtkSmartPointer<vtkPolyData>(Part2::*)(bool)) &Part2::GetVtk, 
            " " , py::arg("includeEdges") = false )
        .def(
            "GetKeyForAttributes", 
            (unsigned int(Part2::*)(::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >)) &Part2::GetKeyForAttributes, 
            " " , py::arg("rAttributes") )
        .def(
            "EdgeHasAttribute", 
            (bool(Part2::*)(::Vertex<2> const &, ::std::string const &)) &Part2::EdgeHasAttribute, 
            " " , py::arg("rLoc"), py::arg("rLabel") )
        .def(
            "IsPointInPart", 
            (bool(Part2::*)(::Vertex<2> const &)) &Part2::IsPointInPart, 
            " " , py::arg("rLoc") )
        .def(
            "IsPointInPart", 
            (::std::vector<bool, std::allocator<bool> >(Part2::*)(::vtkSmartPointer<vtkPoints>)) &Part2::IsPointInPart, 
            " " , py::arg("pPoints") )
        .def(
            "MergeCoincidentVertices", 
            (void(Part2::*)()) &Part2::MergeCoincidentVertices, 
            " "  )
        .def(
            "RotateAboutAxis", 
            (void(Part2::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Part2::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle") )
        .def(
            "SetReferenceLengthScale", 
            (void(Part2::*)(::QLength)) &Part2::SetReferenceLengthScale, 
            " " , py::arg("referenceLength") )
        .def(
            "Translate", 
            (void(Part2::*)(::Vertex<2> const &)) &Part2::Translate, 
            " " , py::arg("vector") )
        .def(
            "Write", 
            (void(Part2::*)(::std::string const &, ::GeometryFormat::Value, bool)) &Part2::Write, 
            " " , py::arg("rFilename"), py::arg("format") = ::GeometryFormat::Value::VTP, py::arg("includeEdges") = false )
    ;
}
