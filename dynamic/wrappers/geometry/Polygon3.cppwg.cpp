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

#include "Polygon3.cppwg.hpp"

namespace py = pybind11;
typedef Polygon<3 > Polygon3;
;

void register_Polygon3_class(py::module &m){
py::class_<Polygon3    >(m, "Polygon3")
        .def(py::init<::std::vector<std::shared_ptr<DimensionalChastePoint<3> >, std::allocator<std::shared_ptr<DimensionalChastePoint<3> > > > >(), py::arg("vertices"))
        .def(py::init<::std::shared_ptr<DimensionalChastePoint<3> > >(), py::arg("pVertex"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Polygon<3> >(*)(::std::vector<std::shared_ptr<DimensionalChastePoint<3> >, std::allocator<std::shared_ptr<DimensionalChastePoint<3> > > >)) &Polygon3::Create, 
            " " , py::arg("vertices"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Polygon<3> >(*)(::std::shared_ptr<DimensionalChastePoint<3> >)) &Polygon3::Create, 
            " " , py::arg("pVertex"))
        .def(
            "AddAttribute", 
            (void(Polygon3::*)(::std::string const &, double)) &Polygon3::AddAttribute, 
            " " , py::arg("rLabel"), py::arg("value"))
        .def(
            "AddAttributeToEdgeIfFound", 
            (bool(Polygon3::*)(::DimensionalChastePoint<3>, ::std::string const &, double)) &Polygon3::AddAttributeToEdgeIfFound, 
            " " , py::arg("loc"), py::arg("rLabel"), py::arg("value"))
        .def(
            "AddAttributeToAllEdges", 
            (void(Polygon3::*)(::std::string const &, double)) &Polygon3::AddAttributeToAllEdges, 
            " " , py::arg("rLabel"), py::arg("value"))
        .def(
            "AddVertices", 
            (void(Polygon3::*)(::std::vector<std::shared_ptr<DimensionalChastePoint<3> >, std::allocator<std::shared_ptr<DimensionalChastePoint<3> > > >)) &Polygon3::AddVertices, 
            " " , py::arg("vertices"))
        .def(
            "AddVertex", 
            (void(Polygon3::*)(::std::shared_ptr<DimensionalChastePoint<3> >)) &Polygon3::AddVertex, 
            " " , py::arg("pVertex"))
        .def(
            "ContainsPoint", 
            (bool(Polygon3::*)(::DimensionalChastePoint<3> const &, double)) &Polygon3::ContainsPoint, 
            " " , py::arg("rLocation"), py::arg("tolerance") = 0.)
        .def(
            "GetBoundingBox", 
            (::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > >(Polygon3::*)()) &Polygon3::GetBoundingBox, 
            " " )
        .def(
            "GetCentroid", 
            (::DimensionalChastePoint<3>(Polygon3::*)()) &Polygon3::GetCentroid, 
            " " )
        .def(
            "GetDistance", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(Polygon3::*)(::DimensionalChastePoint<3> const &)) &Polygon3::GetDistance, 
            " " , py::arg("rLocation"))
        .def(
            "GetDistanceToEdges", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(Polygon3::*)(::DimensionalChastePoint<3> const &)) &Polygon3::GetDistanceToEdges, 
            " " , py::arg("rLocation"))
        .def(
            "GetPlane", 
            (::vtkSmartPointer<vtkPlane>(Polygon3::*)()) &Polygon3::GetPlane, 
            " " )
        .def(
            "GetNormal", 
            (::boost::numeric::ublas::c_vector<double, 3>(Polygon3::*)()) &Polygon3::GetNormal, 
            " " )
        .def(
            "GetVertex", 
            (::std::shared_ptr<DimensionalChastePoint<3> >(Polygon3::*)(unsigned int)) &Polygon3::GetVertex, 
            " " , py::arg("idx"))
        .def(
            "GetVertices", 
            (::std::vector<std::shared_ptr<DimensionalChastePoint<3> >, std::allocator<std::shared_ptr<DimensionalChastePoint<3> > > >(Polygon3::*)()) &Polygon3::GetVertices, 
            " " )
        .def(
            "GetVtkPolygon", 
            (::vtkSmartPointer<vtkPolygon>(Polygon3::*)()) &Polygon3::GetVtkPolygon, 
            " " )
        .def(
            "GetEdgeAttributes", 
            (::std::vector<std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >, std::allocator<std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > > >(Polygon3::*)()) &Polygon3::GetEdgeAttributes, 
            " " )
        .def(
            "GetAttributes", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Polygon3::*)()) &Polygon3::GetAttributes, 
            " " )
        .def(
            "GetVtkVertices", 
            (::std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(Polygon3::*)()) &Polygon3::GetVtkVertices, 
            " " )
        .def(
            "EdgeHasAttribute", 
            (bool(Polygon3::*)(::DimensionalChastePoint<3>, ::std::string const &)) &Polygon3::EdgeHasAttribute, 
            " " , py::arg("loc"), py::arg("rLabel"))
        .def(
            "HasAttribute", 
            (bool(Polygon3::*)(::std::string const &)) &Polygon3::HasAttribute, 
            " " , py::arg("rLabel"))
        .def(
            "ReplaceVertex", 
            (void(Polygon3::*)(unsigned int, ::std::shared_ptr<DimensionalChastePoint<3> >)) &Polygon3::ReplaceVertex, 
            " " , py::arg("idx"), py::arg("pVertex"))
        .def(
            "RotateAboutAxis", 
            (void(Polygon3::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Polygon3::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle"))
        .def(
            "Translate", 
            (void(Polygon3::*)(::DimensionalChastePoint<3>)) &Polygon3::Translate, 
            " " , py::arg("translationVector"))
    ;
}
