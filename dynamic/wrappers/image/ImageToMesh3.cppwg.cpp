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
#include "ImageToMesh.hpp"

#include "PythonObjectConverters.hpp"
#include "ImageToMesh3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef ImageToMesh<3 > ImageToMesh3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_ImageToMesh3_class(py::module &m){
py::class_<ImageToMesh3  , std::shared_ptr<ImageToMesh3 >   >(m, "ImageToMesh3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ImageToMesh<3> >(*)()) &ImageToMesh3::Create, 
            " "  )
        .def(
            "GetMesh", 
            (::std::shared_ptr<DiscreteContinuumMesh<3, 3> >(ImageToMesh3::*)()) &ImageToMesh3::GetMesh, 
            " "  )
        .def(
            "SetElementSize", 
            (void(ImageToMesh3::*)(::QVolume)) &ImageToMesh3::SetElementSize, 
            " " , py::arg("elementSize") )
        .def(
            "GetMeshBoundary", 
            (::vtkSmartPointer<vtkPolyData>(ImageToMesh3::*)()) &ImageToMesh3::GetMeshBoundary, 
            " "  )
        .def(
            "GetMeshHoles", 
            (::std::vector<Vertex<3>, std::allocator<Vertex<3> > >(ImageToMesh3::*)()) &ImageToMesh3::GetMeshHoles, 
            " "  )
        .def(
            "SetInput", 
            (void(ImageToMesh3::*)(::vtkSmartPointer<vtkImageData>)) &ImageToMesh3::SetInput, 
            " " , py::arg("pImage") )
        .def(
            "SetInputRaw", 
            (void(ImageToMesh3::*)(::vtkImageData *)) &ImageToMesh3::SetInputRaw, 
            " " , py::arg("pImage") )
        .def(
            "SetTissueDomain", 
            (void(ImageToMesh3::*)(::std::shared_ptr<Part<3> >)) &ImageToMesh3::SetTissueDomain, 
            " " , py::arg("pTissueDomain") )
        .def(
            "Update", 
            (void(ImageToMesh3::*)()) &ImageToMesh3::Update, 
            " "  )
    ;
}
