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
#include "MultiFormatMeshWriter.hpp"

#include "PythonObjectConverters.hpp"
#include "MultiFormatMeshWriter3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef MultiFormatMeshWriter<3 > MultiFormatMeshWriter3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_MultiFormatMeshWriter3_class(py::module &m){
py::class_<MultiFormatMeshWriter3  , std::shared_ptr<MultiFormatMeshWriter3 >   >(m, "MultiFormatMeshWriter3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MultiFormatMeshWriter<3> >(*)()) &MultiFormatMeshWriter3::Create, 
            " "  )
        .def(
            "SetFileName", 
            (void(MultiFormatMeshWriter3::*)(::std::string const &)) &MultiFormatMeshWriter3::SetFileName, 
            " " , py::arg("rFilename") )
        .def(
            "SetMesh", 
            (void(MultiFormatMeshWriter3::*)(::vtkSmartPointer<vtkUnstructuredGrid>)) &MultiFormatMeshWriter3::SetMesh, 
            " " , py::arg("pMesh") )
        .def(
            "SetMesh", 
            (void(MultiFormatMeshWriter3::*)(::std::shared_ptr<DiscreteContinuumMesh<3, 3> >, bool)) &MultiFormatMeshWriter3::SetMesh, 
            " " , py::arg("pMesh"), py::arg("addBoundaryLabels") = false )
        .def(
            "SetOutputFormat", 
            (void(MultiFormatMeshWriter3::*)(::MeshFormat::Value)) &MultiFormatMeshWriter3::SetOutputFormat, 
            " " , py::arg("outputFormat") )
        .def(
            "Write", 
            (void(MultiFormatMeshWriter3::*)()) &MultiFormatMeshWriter3::Write, 
            " "  )
    ;
}
