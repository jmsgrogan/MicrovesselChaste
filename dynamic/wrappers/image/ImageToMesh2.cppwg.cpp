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
#include "ImageToMesh2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef ImageToMesh<2 > ImageToMesh2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_ImageToMesh2_class(py::module &m){
py::class_<ImageToMesh2  , std::shared_ptr<ImageToMesh2 >   >(m, "ImageToMesh2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ImageToMesh<2> >(*)()) &ImageToMesh2::Create, 
            " "  )
        .def(
            "GetMesh", 
            (::std::shared_ptr<DiscreteContinuumMesh<2, 2> >(ImageToMesh2::*)()) &ImageToMesh2::GetMesh, 
            " "  )
        .def(
            "SetElementSize", 
            (void(ImageToMesh2::*)(::QVolume)) &ImageToMesh2::SetElementSize, 
            " " , py::arg("elementSize") )
        .def(
            "GetMeshBoundary", 
            (::vtkSmartPointer<vtkPolyData>(ImageToMesh2::*)()) &ImageToMesh2::GetMeshBoundary, 
            " "  )
        .def(
            "GetMeshHoles", 
            (::std::vector<Vertex<2>, std::allocator<Vertex<2> > >(ImageToMesh2::*)()) &ImageToMesh2::GetMeshHoles, 
            " "  )
        .def(
            "SetInput", 
            (void(ImageToMesh2::*)(::vtkSmartPointer<vtkImageData>)) &ImageToMesh2::SetInput, 
            " " , py::arg("pImage") )
        .def(
            "SetInputRaw", 
            (void(ImageToMesh2::*)(::vtkImageData *)) &ImageToMesh2::SetInputRaw, 
            " " , py::arg("pImage") )
        .def(
            "SetTissueDomain", 
            (void(ImageToMesh2::*)(::std::shared_ptr<Part<2> >)) &ImageToMesh2::SetTissueDomain, 
            " " , py::arg("pTissueDomain") )
        .def(
            "Update", 
            (void(ImageToMesh2::*)()) &ImageToMesh2::Update, 
            " "  )
    ;
}
