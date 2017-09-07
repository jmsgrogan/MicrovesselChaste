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
#include "RegularGridWriter.hpp"

#include "PythonObjectConverters.hpp"
#include "RegularGridWriter.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef RegularGridWriter RegularGridWriter;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_RegularGridWriter_class(py::module &m){
py::class_<RegularGridWriter  , std::shared_ptr<RegularGridWriter >   >(m, "RegularGridWriter")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<RegularGridWriter>(*)()) &RegularGridWriter::Create, 
            " "  )
        .def(
            "SetFilename", 
            (void(RegularGridWriter::*)(::std::string const &)) &RegularGridWriter::SetFilename, 
            " " , py::arg("rFilename") )
        .def(
            "SetWholeExtents", 
            (void(RegularGridWriter::*)(::std::vector<unsigned int, std::allocator<unsigned int> >)) &RegularGridWriter::SetWholeExtents, 
            " " , py::arg("wholeExtents") )
        .def(
            "SetImage", 
            (void(RegularGridWriter::*)(::vtkSmartPointer<vtkImageData>)) &RegularGridWriter::SetImage, 
            " " , py::arg("pImage") )
        .def(
            "Write", 
            (void(RegularGridWriter::*)()) &RegularGridWriter::Write, 
            " "  )
    ;
}
