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

#include "Part3.cppwg.hpp"

namespace py = pybind11;
typedef Part<3 > Part3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_Part3_class(py::module &m){
py::class_<Part3  , std::shared_ptr<Part3 >   >(m, "Part3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<Part<3> >(*)()) &Part3::Create, 
            " "  )
        .def(
            "AddAttribute", 
            (void(Part3::*)(::std::string const &, double)) &Part3::AddAttribute, 
            " " , py::arg("rLabel"), py::arg("value") )
        .def(
            "AddAttributeToEdgeIfFound", 
            (void(Part3::*)(::Vertex<3> const &, ::std::string const &, double)) &Part3::AddAttributeToEdgeIfFound, 
            " " , py::arg("rLoc"), py::arg("rLabel"), py::arg("value") )
        .def(
            "AddAttributeToPolygons", 
            (void(Part3::*)(::std::string const &, double)) &Part3::AddAttributeToPolygons, 
            " " , py::arg("rLabel"), py::arg("value") )
        .def(
            "AddAttributeToPolygonIfFound", 
            (void(Part3::*)(::Vertex<3> const &, ::std::string const &, double)) &Part3::AddAttributeToPolygonIfFound, 
            " " , py::arg("rLoc"), py::arg("rLabel"), py::arg("value") )
        .def(
            "AddCircle", 
            (::std::shared_ptr<Polygon<3> >(Part3::*)(::QLength, ::Vertex<3>, unsigned int)) &Part3::AddCircle, 
            " " , py::arg("radius"), py::arg("centre") = Vertex<3>(), py::arg("numSegments") = 24 )
        .def(
            "AddCylinder", 
            (void(Part3::*)(::QLength, ::QLength, ::Vertex<3>, unsigned int)) &Part3::AddCylinder, 
            " " , py::arg("radius"), py::arg("depth"), py::arg("centre") = Vertex<3>(), py::arg("numSegments") = 24 )
        .def(
            "AddCuboid", 
            (void(Part3::*)(::QLength, ::QLength, ::QLength, ::Vertex<3>)) &Part3::AddCuboid, 
            " " , py::arg("sizeX"), py::arg("sizeY"), py::arg("sizeZ"), py::arg("origin") = Vertex<3>() )
        .def(
            "AddHoleMarker", 
            (void(Part3::*)(::Vertex<3>)) &Part3::AddHoleMarker, 
            " " , py::arg("location") )
        .def(
            "AddRegionMarker", 
            (void(Part3::*)(::Vertex<3>, unsigned int)) &Part3::AddRegionMarker, 
            " " , py::arg("location"), py::arg("value") )
        .def(
            "AddPolygon", 
            (::std::shared_ptr<Polygon<3> >(Part3::*)(::std::vector<std::shared_ptr<Vertex<3> >, std::allocator<std::shared_ptr<Vertex<3> > > > const &, bool, ::std::shared_ptr<Facet<3> >)) &Part3::AddPolygon, 
            " " , py::arg("vertices"), py::arg("newFacet") = false, py::arg("pFacet") = FacetPtr<3>() )
        .def(
            "AddPolygon", 
            (::std::shared_ptr<Polygon<3> >(Part3::*)(::std::shared_ptr<Polygon<3> >, bool, ::std::shared_ptr<Facet<3> >)) &Part3::AddPolygon, 
            " " , py::arg("pPolygon"), py::arg("newFacet") = false, py::arg("pFacet") = FacetPtr<3>() )
        .def(
            "AddRectangle", 
            (::std::shared_ptr<Polygon<3> >(Part3::*)(::QLength, ::QLength, ::Vertex<3>)) &Part3::AddRectangle, 
            " " , py::arg("sizeX"), py::arg("sizeY"), py::arg("origin") = Vertex<3>() )
        .def(
            "AddVesselNetwork", 
            (void(Part3::*)(::std::shared_ptr<VesselNetwork<3> >, bool, bool)) &Part3::AddVesselNetwork, 
            " " , py::arg("pVesselNetwork"), py::arg("surface") = false, py::arg("removeVesselRegion") = true )
        .def(
            "AppendPart", 
            (void(Part3::*)(::std::shared_ptr<Part<3> >)) &Part3::AppendPart, 
            " " , py::arg("pPart") )
        .def(
            "BooleanWithNetwork", 
            (void(Part3::*)(::std::shared_ptr<VesselNetwork<3> >)) &Part3::BooleanWithNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "Extrude", 
            (void(Part3::*)(::std::shared_ptr<Polygon<3> >, ::QLength)) &Part3::Extrude, 
            " " , py::arg("pPolygon"), py::arg("distance") )
        .def(
            "GetBoundingBox", 
            (::std::array<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 6>(Part3::*)()) &Part3::GetBoundingBox, 
            " "  )
        .def(
            "GetContainingGridIndices", 
            (::std::vector<unsigned int, std::allocator<unsigned int> >(Part3::*)(unsigned int, unsigned int, unsigned int, ::QLength)) &Part3::GetContainingGridIndices, 
            " " , py::arg("num_x"), py::arg("num_y"), py::arg("num_z"), py::arg("spacing") )
        .def(
            "GetHoleMarkers", 
            (::std::vector<Vertex<3>, std::allocator<Vertex<3> > >(Part3::*)()) &Part3::GetHoleMarkers, 
            " "  )
        .def(
            "GetRegionMarkers", 
            (::std::vector<std::pair<Vertex<3>, unsigned int>, std::allocator<std::pair<Vertex<3>, unsigned int> > >(Part3::*)()) &Part3::GetRegionMarkers, 
            " "  )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Part3::*)()) &Part3::GetAttributes, 
            " "  )
        .def(
            "GetAttributesKeysForMesh", 
            (::std::map<unsigned int, std::basic_string<char>, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, std::basic_string<char> > > >(Part3::*)(bool)) &Part3::GetAttributesKeysForMesh, 
            " " , py::arg("update") = true )
        .def(
            "GetFacets", 
            (::std::vector<std::shared_ptr<Facet<3> >, std::allocator<std::shared_ptr<Facet<3> > > >(Part3::*)()) &Part3::GetFacets, 
            " "  )
        .def(
            "GetFacet", 
            (::std::shared_ptr<Facet<3> >(Part3::*)(::Vertex<3> const &)) &Part3::GetFacet, 
            " " , py::arg("rLocation") )
        .def(
            "GetPolygons", 
            (::std::vector<std::shared_ptr<Polygon<3> >, std::allocator<std::shared_ptr<Polygon<3> > > >(Part3::*)()) &Part3::GetPolygons, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(Part3::*)()) &Part3::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetSegmentIndices", 
            (::std::vector<std::pair<std::pair<unsigned int, unsigned int>, unsigned int>, std::allocator<std::pair<std::pair<unsigned int, unsigned int>, unsigned int> > >(Part3::*)()) &Part3::GetSegmentIndices, 
            " "  )
        .def(
            "GetVertices", 
            (::std::vector<std::shared_ptr<Vertex<3> >, std::allocator<std::shared_ptr<Vertex<3> > > >(Part3::*)()) &Part3::GetVertices, 
            " "  )
        .def(
            "GetVtk", 
            (::vtkSmartPointer<vtkPolyData>(Part3::*)(bool)) &Part3::GetVtk, 
            " " , py::arg("includeEdges") = false )
        .def(
            "GetKeyForAttributes", 
            (unsigned int(Part3::*)(::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >)) &Part3::GetKeyForAttributes, 
            " " , py::arg("rAttributes") )
        .def(
            "EdgeHasAttribute", 
            (bool(Part3::*)(::Vertex<3> const &, ::std::string const &)) &Part3::EdgeHasAttribute, 
            " " , py::arg("rLoc"), py::arg("rLabel") )
        .def(
            "IsPointInPart", 
            (bool(Part3::*)(::Vertex<3> const &)) &Part3::IsPointInPart, 
            " " , py::arg("rLoc") )
        .def(
            "IsPointInPart", 
            (::std::vector<bool, std::allocator<bool> >(Part3::*)(::vtkSmartPointer<vtkPoints>)) &Part3::IsPointInPart, 
            " " , py::arg("pPoints") )
        .def(
            "MergeCoincidentVertices", 
            (void(Part3::*)()) &Part3::MergeCoincidentVertices, 
            " "  )
        .def(
            "RotateAboutAxis", 
            (void(Part3::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Part3::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle") )
        .def(
            "SetReferenceLengthScale", 
            (void(Part3::*)(::QLength)) &Part3::SetReferenceLengthScale, 
            " " , py::arg("referenceLength") )
        .def(
            "Translate", 
            (void(Part3::*)(::Vertex<3> const &)) &Part3::Translate, 
            " " , py::arg("vector") )
        .def(
            "Write", 
            (void(Part3::*)(::std::string const &, ::GeometryFormat::Value, bool)) &Part3::Write, 
            " " , py::arg("rFilename"), py::arg("format") = ::GeometryFormat::Value::VTP, py::arg("includeEdges") = false )
    ;
}
