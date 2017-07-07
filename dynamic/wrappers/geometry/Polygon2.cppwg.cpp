#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Polygon.hpp"

#include "Polygon2.cppwg.hpp"

namespace py = pybind11;
typedef Polygon<2 > Polygon2;
;

void register_Polygon2_class(py::module &m){
py::class_<Polygon2    >(m, "Polygon2")
        .def(py::init<::std::vector<std::shared_ptr<DimensionalChastePoint<2> >, std::allocator<std::shared_ptr<DimensionalChastePoint<2> > > > >(), py::arg("vertices"))
        .def(py::init<::std::shared_ptr<DimensionalChastePoint<2> > >(), py::arg("pVertex"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Polygon<2> >(*)(::std::vector<std::shared_ptr<DimensionalChastePoint<2> >, std::allocator<std::shared_ptr<DimensionalChastePoint<2> > > >)) &Polygon2::Create, 
            " " , py::arg("vertices") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Polygon<2> >(*)(::std::shared_ptr<DimensionalChastePoint<2> >)) &Polygon2::Create, 
            " " , py::arg("pVertex") )
        .def(
            "AddAttribute", 
            (void(Polygon2::*)(::std::string const &, double)) &Polygon2::AddAttribute, 
            " " , py::arg("rLabel"), py::arg("value") )
        .def(
            "AddAttributeToEdgeIfFound", 
            (bool(Polygon2::*)(::DimensionalChastePoint<2>, ::std::string const &, double)) &Polygon2::AddAttributeToEdgeIfFound, 
            " " , py::arg("loc"), py::arg("rLabel"), py::arg("value") )
        .def(
            "AddAttributeToAllEdges", 
            (void(Polygon2::*)(::std::string const &, double)) &Polygon2::AddAttributeToAllEdges, 
            " " , py::arg("rLabel"), py::arg("value") )
        .def(
            "AddVertices", 
            (void(Polygon2::*)(::std::vector<std::shared_ptr<DimensionalChastePoint<2> >, std::allocator<std::shared_ptr<DimensionalChastePoint<2> > > >)) &Polygon2::AddVertices, 
            " " , py::arg("vertices") )
        .def(
            "AddVertex", 
            (void(Polygon2::*)(::std::shared_ptr<DimensionalChastePoint<2> >)) &Polygon2::AddVertex, 
            " " , py::arg("pVertex") )
        .def(
            "ContainsPoint", 
            (bool(Polygon2::*)(::DimensionalChastePoint<2> const &, double)) &Polygon2::ContainsPoint, 
            " " , py::arg("rLocation"), py::arg("tolerance") = 0. )
        .def(
            "GetBoundingBox", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(Polygon2::*)()) &Polygon2::GetBoundingBox, 
            " "  )
        .def(
            "GetCentroid", 
            (::DimensionalChastePoint<2>(Polygon2::*)()) &Polygon2::GetCentroid, 
            " "  )
        .def(
            "GetDistance", 
            (::QLength(Polygon2::*)(::DimensionalChastePoint<2> const &)) &Polygon2::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetDistanceToEdges", 
            (::QLength(Polygon2::*)(::DimensionalChastePoint<2> const &)) &Polygon2::GetDistanceToEdges, 
            " " , py::arg("rLocation") )
        .def(
            "GetPlane", 
            (::vtkSmartPointer<vtkPlane>(Polygon2::*)()) &Polygon2::GetPlane, 
            " "  )
        .def(
            "GetNormal", 
            (::boost::numeric::ublas::c_vector<double, 2>(Polygon2::*)()) &Polygon2::GetNormal, 
            " "  )
        .def(
            "GetVertex", 
            (::std::shared_ptr<DimensionalChastePoint<2> >(Polygon2::*)(unsigned int)) &Polygon2::GetVertex, 
            " " , py::arg("idx") )
        .def(
            "GetVertices", 
            (::std::vector<std::shared_ptr<DimensionalChastePoint<2> >, std::allocator<std::shared_ptr<DimensionalChastePoint<2> > > >(Polygon2::*)()) &Polygon2::GetVertices, 
            " "  )
        .def(
            "GetVtkPolygon", 
            (::vtkSmartPointer<vtkPolygon>(Polygon2::*)()) &Polygon2::GetVtkPolygon, 
            " "  )
        .def(
            "GetEdgeAttributes", 
            (::std::vector<std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >, std::allocator<std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > > >(Polygon2::*)()) &Polygon2::GetEdgeAttributes, 
            " "  )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Polygon2::*)()) &Polygon2::GetAttributes, 
            " "  )
        .def(
            "GetVtkVertices", 
            (::std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(Polygon2::*)()) &Polygon2::GetVtkVertices, 
            " "  )
        .def(
            "EdgeHasAttribute", 
            (bool(Polygon2::*)(::DimensionalChastePoint<2>, ::std::string const &)) &Polygon2::EdgeHasAttribute, 
            " " , py::arg("loc"), py::arg("rLabel") )
        .def(
            "HasAttribute", 
            (bool(Polygon2::*)(::std::string const &)) &Polygon2::HasAttribute, 
            " " , py::arg("rLabel") )
        .def(
            "ReplaceVertex", 
            (void(Polygon2::*)(unsigned int, ::std::shared_ptr<DimensionalChastePoint<2> >)) &Polygon2::ReplaceVertex, 
            " " , py::arg("idx"), py::arg("pVertex") )
        .def(
            "RotateAboutAxis", 
            (void(Polygon2::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Polygon2::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle") )
        .def(
            "Translate", 
            (void(Polygon2::*)(::DimensionalChastePoint<2>)) &Polygon2::Translate, 
            " " , py::arg("translationVector") )
    ;
}
