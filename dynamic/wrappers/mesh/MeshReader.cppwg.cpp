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
#include "MeshReader.hpp"

#include "MeshReader.cppwg.hpp"

namespace py = pybind11;
typedef MeshReader MeshReader;
;

void register_MeshReader_class(py::module &m){
py::class_<MeshReader    >(m, "MeshReader")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MeshReader>(*)()) &MeshReader::Create, 
            " "  )
        .def(
            "GetMesh", 
            (::vtkSmartPointer<vtkUnstructuredGrid>(MeshReader::*)()) &MeshReader::GetMesh, 
            " "  )
        .def(
            "SetFileName", 
            (void(MeshReader::*)(::std::string const &)) &MeshReader::SetFileName, 
            " " , py::arg("rFilename") )
        .def(
            "Read", 
            (void(MeshReader::*)()) &MeshReader::Read, 
            " "  )
    ;
}
