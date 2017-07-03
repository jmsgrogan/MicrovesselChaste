#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "ImageToMesh.hpp"

#include "ImageToMesh2.cppwg.hpp"

namespace py = pybind11;
typedef ImageToMesh<2 > ImageToMesh2;
;

void register_ImageToMesh2_class(py::module &m){
py::class_<ImageToMesh2    >(m, "ImageToMesh2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ImageToMesh<2> >(*)()) &ImageToMesh2::Create, 
            " " )
        .def(
            "GetMesh", 
            (::std::shared_ptr<DiscreteContinuumMesh<2, 2> >(ImageToMesh2::*)()) &ImageToMesh2::GetMesh, 
            " " )
        .def(
            "SetElementSize", 
            (void(ImageToMesh2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<3, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &ImageToMesh2::SetElementSize, 
            " " , py::arg("elementSize"))
        .def(
            "GetMeshBoundary", 
            (::vtkSmartPointer<vtkPolyData>(ImageToMesh2::*)()) &ImageToMesh2::GetMeshBoundary, 
            " " )
        .def(
            "GetMeshHoles", 
            (::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >(ImageToMesh2::*)()) &ImageToMesh2::GetMeshHoles, 
            " " )
        .def(
            "SetInput", 
            (void(ImageToMesh2::*)(::vtkSmartPointer<vtkImageData>)) &ImageToMesh2::SetInput, 
            " " , py::arg("pImage"))
        .def(
            "SetInputRaw", 
            (void(ImageToMesh2::*)(::vtkImageData *)) &ImageToMesh2::SetInputRaw, 
            " " , py::arg("pImage"))
        .def(
            "SetTissueDomain", 
            (void(ImageToMesh2::*)(::std::shared_ptr<Part<2> >)) &ImageToMesh2::SetTissueDomain, 
            " " , py::arg("pTissueDomain"))
        .def(
            "Update", 
            (void(ImageToMesh2::*)()) &ImageToMesh2::Update, 
            " " )
    ;
}
