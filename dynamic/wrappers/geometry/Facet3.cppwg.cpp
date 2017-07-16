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

#include "Facet3.cppwg.hpp"

namespace py = pybind11;
typedef Facet<3 > Facet3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_Facet3_class(py::module &m){
py::class_<Facet3  , std::shared_ptr<Facet3 >   >(m, "Facet3")
        .def(py::init<::std::vector<std::shared_ptr<Polygon<3> >, std::allocator<std::shared_ptr<Polygon<3> > > > >(), py::arg("polygons"))
        .def(py::init<::std::shared_ptr<Polygon<3> > >(), py::arg("pPolygon"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Facet<3> >(*)(::std::vector<std::shared_ptr<Polygon<3> >, std::allocator<std::shared_ptr<Polygon<3> > > >)) &Facet3::Create, 
            " " , py::arg("polygons") )
        .def_static(
            "Create", 
            (::std::shared_ptr<Facet<3> >(*)(::std::shared_ptr<Polygon<3> >)) &Facet3::Create, 
            " " , py::arg("pPolygon") )
        .def(
            "AddPolygons", 
            (void(Facet3::*)(::std::vector<std::shared_ptr<Polygon<3> >, std::allocator<std::shared_ptr<Polygon<3> > > >)) &Facet3::AddPolygons, 
            " " , py::arg("polygons") )
        .def(
            "AddPolygon", 
            (void(Facet3::*)(::std::shared_ptr<Polygon<3> >)) &Facet3::AddPolygon, 
            " " , py::arg("pPolygon") )
        .def(
            "ContainsPoint", 
            (bool(Facet3::*)(::Vertex<3> const &)) &Facet3::ContainsPoint, 
            " " , py::arg("rLocation") )
        .def(
            "GetBoundingBox", 
            (::std::array<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, 6>(Facet3::*)()) &Facet3::GetBoundingBox, 
            " "  )
        .def(
            "GetCentroid", 
            (::Vertex<3>(Facet3::*)()) &Facet3::GetCentroid, 
            " "  )
        .def(
            "GetDistance", 
            (::QLength(Facet3::*)(::Vertex<3> const &)) &Facet3::GetDistance, 
            " " , py::arg("rLocation") )
        .def(
            "GetPlane", 
            (::vtkSmartPointer<vtkPlane>(Facet3::*)()) &Facet3::GetPlane, 
            " "  )
        .def(
            "GetNormal", 
            (::boost::numeric::ublas::c_vector<double, 3>(Facet3::*)()) &Facet3::GetNormal, 
            " "  )
        .def(
            "GetPolygons", 
            (::std::vector<std::shared_ptr<Polygon<3> >, std::allocator<std::shared_ptr<Polygon<3> > > >(Facet3::*)()) &Facet3::GetPolygons, 
            " "  )
        .def(
            "rGetVertices", 
            (::std::vector<std::shared_ptr<Vertex<3> >, std::allocator<std::shared_ptr<Vertex<3> > > > const &(Facet3::*)()) &Facet3::rGetVertices, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "GetVtkVertices", 
            (::std::pair<vtkSmartPointer<vtkPoints>, vtkSmartPointer<vtkIdTypeArray> >(Facet3::*)()) &Facet3::GetVtkVertices, 
            " "  )
        .def(
            "RotateAboutAxis", 
            (void(Facet3::*)(::boost::numeric::ublas::c_vector<double, 3>, double)) &Facet3::RotateAboutAxis, 
            " " , py::arg("axis"), py::arg("angle") )
        .def(
            "Translate", 
            (void(Facet3::*)(::Vertex<3> const &)) &Facet3::Translate, 
            " " , py::arg("rTranslationVector") )
        .def(
            "UpdateVertices", 
            (void(Facet3::*)()) &Facet3::UpdateVertices, 
            " "  )
    ;
}
