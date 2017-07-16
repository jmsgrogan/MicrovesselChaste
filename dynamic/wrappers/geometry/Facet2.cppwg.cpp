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
#include "Facet.hpp"

#include "Facet2.cppwg.hpp"

namespace py = pybind11;
typedef Facet<2 > Facet2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_Facet2_class(py::module &m){
py::class_<Facet2  , std::shared_ptr<Facet2 >   >(m, "Facet2")
        .def(py::init<::std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > > >(), py::arg("polygons"))
        .def(py::init<::std::shared_ptr<Polygon<2> > >(), py::arg("pPolygon"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Facet<2> >(*)(::std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > >)) &Facet2::Create, 
            " " , py::arg("polygons") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Facet<2> >(*)(::std::shared_ptr<Polygon<2> >)) &Facet2::Create, 
            " " , py::arg("pPolygon") )
        .def(
            "AddPolygons", 
            (void(Facet2::*)(::std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > >)) &Facet2::AddPolygons, 
            " " , py::arg("polygons") )
        .def(
            "AddPolygon", 
            (void(Facet2::*)(::std::shared_ptr<Polygon<2> >)) &Facet2::AddPolygon, 
            " " , py::arg("pPolygon") )
        .def(
            "ContainsPoint", 
            (bool(Facet2::*)(::Vertex<2> const &)) &Facet2::ContainsPoint, 
            " " , py::arg("rLocation") )
        .def(
            "GetBoundingBox", 
            (::std::array<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 6>(Facet2::*)()) &Facet2::GetBoundingBox, 
            " "  )
        .def(
            "GetCentroid", 
            (::Vertex<2>(Facet2::*)()) &Facet2::GetCentroid, 
            " "  )
        .def(
            "GetDistance", 
            (::QLength(Facet2::*)(::Vertex<2> const &)) &Facet2::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetPlane", 
            (::vtkSmartPointer<vtkPlane>(Facet2::*)()) &Facet2::GetPlane, 
            " "  )
        .def(
            "GetNormal", 
            (::boost::numeric::ublas::c_vector<double, 3>(Facet2::*)()) &Facet2::GetNormal, 
            " "  )
        .def(
            "GetPolygons", 
            (::std::vector<std::shared_ptr<Polygon<2> >, std::allocator<std::shared_ptr<Polygon<2> > > >(Facet2::*)()) &Facet2::GetPolygons, 
            " "  )
        .def(
            "rGetVertices", 
            (::std::vector<std::shared_ptr<Vertex<2> >, std::allocator<std::shared_ptr<Vertex<2> > > > const &(Facet2::*)()) &Facet2::rGetVertices, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "GetVtkVertices", 
            (::std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(Facet2::*)()) &Facet2::GetVtkVertices, 
            " "  )
        .def(
            "RotateAboutAxis", 
            (void(Facet2::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Facet2::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle") )
        .def(
            "Translate", 
            (void(Facet2::*)(::Vertex<2> const &)) &Facet2::Translate, 
            " " , py::arg("rTranslationVector") )
        .def(
            "UpdateVertices", 
            (void(Facet2::*)()) &Facet2::UpdateVertices, 
            " "  )
    ;
}
