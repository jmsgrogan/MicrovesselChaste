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

#include "MultiFormatMeshWriter2.cppwg.hpp"

namespace py = pybind11;
typedef MultiFormatMeshWriter<2 > MultiFormatMeshWriter2;
;

void register_MultiFormatMeshWriter2_class(py::module &m){
py::class_<MultiFormatMeshWriter2    >(m, "MultiFormatMeshWriter2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MultiFormatMeshWriter<2> >(*)()) &MultiFormatMeshWriter2::Create, 
            " "  )
        .def(
            "SetFileName", 
            (void(MultiFormatMeshWriter2::*)(::std::string const &)) &MultiFormatMeshWriter2::SetFileName, 
            " " , py::arg("rFilename") )
        .def(
            "SetMesh", 
            (void(MultiFormatMeshWriter2::*)(::vtkSmartPointer<vtkUnstructuredGrid>)) &MultiFormatMeshWriter2::SetMesh, 
            " " , py::arg("pMesh") )
        .def(
            "SetMesh", 
            (void(MultiFormatMeshWriter2::*)(::std::shared_ptr<DiscreteContinuumMesh<2, 2> >, bool)) &MultiFormatMeshWriter2::SetMesh, 
            " " , py::arg("pMesh"), py::arg("addBoundaryLabels") = false )
        .def(
            "SetOutputFormat", 
            (void(MultiFormatMeshWriter2::*)(::MeshFormat::Value)) &MultiFormatMeshWriter2::SetOutputFormat, 
            " " , py::arg("outputFormat") )
        .def(
            "Write", 
            (void(MultiFormatMeshWriter2::*)()) &MultiFormatMeshWriter2::Write, 
            " "  )
    ;
}
