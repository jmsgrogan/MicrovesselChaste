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
#include "VesselNetworkGenerator.hpp"

#include "PythonObjectConverters.hpp"
#include "VesselNetworkGenerator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef VesselNetworkGenerator<3 > VesselNetworkGenerator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkGenerator3_class(py::module &m){
py::class_<VesselNetworkGenerator3  , std::shared_ptr<VesselNetworkGenerator3 >   >(m, "VesselNetworkGenerator3")
        .def(py::init< >())
        .def(
            "GenerateParallelNetwork", 
            (::std::shared_ptr<VesselNetwork<3> >(VesselNetworkGenerator3::*)(::std::shared_ptr<Part<3> >, ::QPerArea, ::VesselDistribution::Value, ::QLength, bool, ::std::vector<std::shared_ptr<Vertex<3> >, std::allocator<std::shared_ptr<Vertex<3> > > >)) &VesselNetworkGenerator3::GenerateParallelNetwork, 
            " " , py::arg("domain"), py::arg("targetDensity"), py::arg("distrbutionType"), py::arg("exclusionDistance") = 0_m, py::arg("useBbox") = false, py::arg("seeds") = std::vector<std::shared_ptr<Vertex<3> > >() )
        .def(
            "GenerateHexagonalNetwork", 
            (::std::shared_ptr<VesselNetwork<3> >(VesselNetworkGenerator3::*)(::QLength, ::QLength, ::QLength, bool)) &VesselNetworkGenerator3::GenerateHexagonalNetwork, 
            " " , py::arg("width"), py::arg("height"), py::arg("vesselLength"), py::arg("fillDomain") = false )
        .def(
            "GenerateHexagonalUnit", 
            (::std::shared_ptr<VesselNetwork<3> >(VesselNetworkGenerator3::*)(::QLength)) &VesselNetworkGenerator3::GenerateHexagonalUnit, 
            " " , py::arg("vesselLength") )
        .def(
            "GenerateBifurcationUnit", 
            (::std::shared_ptr<VesselNetwork<3> >(VesselNetworkGenerator3::*)(::QLength, ::Vertex<3>)) &VesselNetworkGenerator3::GenerateBifurcationUnit, 
            " " , py::arg("vesselLength"), py::arg("startPosition") = Vertex<3>() )
        .def(
            "GenerateSingleVessel", 
            (::std::shared_ptr<VesselNetwork<3> >(VesselNetworkGenerator3::*)(::QLength, ::Vertex<3>, unsigned int, unsigned int)) &VesselNetworkGenerator3::GenerateSingleVessel, 
            " " , py::arg("vesselLength"), py::arg("startPosition") = Vertex<3>(), py::arg("divisions") = 0, py::arg("axis") = 2 )
        .def(
            "GenerateOvalNetwork", 
            (::std::shared_ptr<VesselNetwork<3> >(VesselNetworkGenerator3::*)(::QLength, unsigned int, double, double)) &VesselNetworkGenerator3::GenerateOvalNetwork, 
            " " , py::arg("scaleFactor"), py::arg("num_increments") = 40, py::arg("a_param") = 0.5, py::arg("b_param") = 1. )
        .def(
            "GenerateFromPart", 
            (::std::shared_ptr<VesselNetwork<3> >(VesselNetworkGenerator3::*)(::std::shared_ptr<Part<3> >)) &VesselNetworkGenerator3::GenerateFromPart, 
            " " , py::arg("pPart") )
        .def(
            "PatternUnitByTranslation", 
            (void(VesselNetworkGenerator3::*)(::std::shared_ptr<VesselNetwork<3> >, ::std::array<unsigned int, 3>)) &VesselNetworkGenerator3::PatternUnitByTranslation, 
            " " , py::arg("pInputUnit"), py::arg("numberOfUnits") )
        .def(
            "MapToSphere", 
            (void(VesselNetworkGenerator3::*)(::std::shared_ptr<VesselNetwork<3> >, ::QLength, ::QLength, double, double)) &VesselNetworkGenerator3::MapToSphere, 
            " " , py::arg("pInputUnit"), py::arg("radius"), py::arg("thickess"), py::arg("azimuthExtent"), py::arg("polarExtent") )
        .def(
            "SetReferenceLengthScale", 
            (void(VesselNetworkGenerator3::*)(::QLength)) &VesselNetworkGenerator3::SetReferenceLengthScale, 
            " " , py::arg("rReferenceLength") )
    ;
}
