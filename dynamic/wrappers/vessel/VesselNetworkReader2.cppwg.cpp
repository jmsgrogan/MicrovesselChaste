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
#include "VesselNetworkReader.hpp"

#include "PythonObjectConverters.hpp"
#include "VesselNetworkReader2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VesselNetworkReader<2 > VesselNetworkReader2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkReader2_class(py::module &m){
py::class_<VesselNetworkReader2  , std::shared_ptr<VesselNetworkReader2 >   >(m, "VesselNetworkReader2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkReader<2> >(*)()) &VesselNetworkReader2::Create, 
            " "  )
        .def(
            "Read", 
            (::std::shared_ptr<VesselNetwork<2> >(VesselNetworkReader2::*)()) &VesselNetworkReader2::Read, 
            " "  )
        .def(
            "SetRadiusArrayName", 
            (void(VesselNetworkReader2::*)(::std::string const &)) &VesselNetworkReader2::SetRadiusArrayName, 
            " " , py::arg("rRadius") )
        .def(
            "SetMergeCoincidentPoints", 
            (void(VesselNetworkReader2::*)(bool)) &VesselNetworkReader2::SetMergeCoincidentPoints, 
            " " , py::arg("mergePoints") )
        .def(
            "SetTargetSegmentLength", 
            (void(VesselNetworkReader2::*)(::QLength)) &VesselNetworkReader2::SetTargetSegmentLength, 
            " " , py::arg("targetSegmentLength") )
        .def(
            "SetFileName", 
            (void(VesselNetworkReader2::*)(::std::string const &)) &VesselNetworkReader2::SetFileName, 
            " " , py::arg("rFileName") )
        .def(
            "SetReferenceLengthScale", 
            (void(VesselNetworkReader2::*)(::QLength)) &VesselNetworkReader2::SetReferenceLengthScale, 
            " " , py::arg("rReferenceLength") )
    ;
}
