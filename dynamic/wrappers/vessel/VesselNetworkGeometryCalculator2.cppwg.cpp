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
#include "VesselNetworkGeometryCalculator.hpp"

#include "VesselNetworkGeometryCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef VesselNetworkGeometryCalculator<2 > VesselNetworkGeometryCalculator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_VesselNetworkGeometryCalculator2_class(py::module &m){
py::class_<VesselNetworkGeometryCalculator2  , std::shared_ptr<VesselNetworkGeometryCalculator2 >   >(m, "VesselNetworkGeometryCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselNetworkGeometryCalculator<2> >(*)()) &VesselNetworkGeometryCalculator2::Create, 
            " "  )
        .def_static(
            "GetDistanceToNearestNode", 
            (::QLength(*)(::std::shared_ptr<VesselNetwork<2> >, ::Vertex<2> const &)) &VesselNetworkGeometryCalculator2::GetDistanceToNearestNode, 
            " " , py::arg("pNetwork"), py::arg("rLocation") )
        .def_static(
            "GetNearestNode", 
            (::std::shared_ptr<VesselNode<2> >(*)(::std::shared_ptr<VesselNetwork<2> >, ::Vertex<2> const &)) &VesselNetworkGeometryCalculator2::GetNearestNode, 
            " " , py::arg("pNetwork"), py::arg("rLocation") )
        .def_static(
            "GetNearestNode", 
            (::std::shared_ptr<VesselNode<2> >(*)(::std::shared_ptr<VesselNetwork<2> >, ::std::shared_ptr<VesselNode<2> >)) &VesselNetworkGeometryCalculator2::GetNearestNode, 
            " " , py::arg("pNetwork"), py::arg("pInputNode") )
        .def_static(
            "GetNearestSegment", 
            (::std::pair<std::shared_ptr<VesselSegment<2> >, RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> > >(*)(::std::shared_ptr<VesselNetwork<2> >, ::std::shared_ptr<VesselSegment<2> >)) &VesselNetworkGeometryCalculator2::GetNearestSegment, 
            " " , py::arg("pNetwork"), py::arg("pSegment") )
        .def_static(
            "GetNearestSegment", 
            (::QLength(*)(::std::shared_ptr<VesselNetwork<2> >, ::std::shared_ptr<VesselNode<2> >, ::std::shared_ptr<VesselSegment<2> > &, bool, ::QLength)) &VesselNetworkGeometryCalculator2::GetNearestSegment, 
            " " , py::arg("pNetwork"), py::arg("pNode"), py::arg("pEmptySegment"), py::arg("sameVessel") = true, py::arg("radius") = 0. * unit::metres )
        .def_static(
            "GetNearestSegmentNonVtk", 
            (::QLength(*)(::std::shared_ptr<VesselNetwork<2> >, ::std::shared_ptr<VesselNode<2> >, ::std::shared_ptr<VesselSegment<2> > &, bool)) &VesselNetworkGeometryCalculator2::GetNearestSegmentNonVtk, 
            " " , py::arg("pNetwork"), py::arg("pNode"), py::arg("pEmptySegment"), py::arg("sameVessel") = true )
        .def_static(
            "GetNearestSegment", 
            (::std::pair<std::shared_ptr<VesselSegment<2> >, RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> > >(*)(::std::shared_ptr<VesselNetwork<2> >, ::Vertex<2> const &)) &VesselNetworkGeometryCalculator2::GetNearestSegment, 
            " " , py::arg("pNetwork"), py::arg("rLocation") )
        .def_static(
            "GetNearestVessel", 
            (::std::shared_ptr<Vessel<2> >(*)(::std::shared_ptr<VesselNetwork<2> >, ::Vertex<2> const &)) &VesselNetworkGeometryCalculator2::GetNearestVessel, 
            " " , py::arg("pNetwork"), py::arg("rLocation") )
        .def_static(
            "GetInterCapillaryDistances", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkGeometryCalculator2::GetInterCapillaryDistances, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetTotalLength", 
            (::QLength(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkGeometryCalculator2::GetTotalLength, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetTotalVolume", 
            (::QVolume(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkGeometryCalculator2::GetTotalVolume, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetTotalSurfaceArea", 
            (::QArea(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkGeometryCalculator2::GetTotalSurfaceArea, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetAverageInterSegmentDistance", 
            (::QLength(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkGeometryCalculator2::GetAverageInterSegmentDistance, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetAverageVesselLength", 
            (::QLength(*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkGeometryCalculator2::GetAverageVesselLength, 
            " " , py::arg("pNetwork") )
        .def_static(
            "GetVesselLengthDistribution", 
            (::std::vector<unsigned int, std::allocator<unsigned int> >(*)(::std::shared_ptr<VesselNetwork<2> >, double, unsigned int)) &VesselNetworkGeometryCalculator2::GetVesselLengthDistribution, 
            " " , py::arg("pNetwork"), py::arg("binSpacing") = 10., py::arg("numberOfBins") = 10 )
        .def_static(
            "GetNumberOfNodesNearLocation", 
            (unsigned int(*)(::std::shared_ptr<VesselNetwork<2> >, ::Vertex<2> const &, double)) &VesselNetworkGeometryCalculator2::GetNumberOfNodesNearLocation, 
            " " , py::arg("pNetwork"), py::arg("rLocation"), py::arg("tolerance") = 0. )
        .def_static(
            "GetNodesInSphere", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(*)(::std::shared_ptr<VesselNetwork<2> >, ::Vertex<2> const &, ::QLength)) &VesselNetworkGeometryCalculator2::GetNodesInSphere, 
            " " , py::arg("pNetwork"), py::arg("rCentre"), py::arg("radius") )
        .def_static(
            "GetExtents", 
            (::std::pair<Vertex<2>, Vertex<2> >(*)(::std::shared_ptr<VesselNetwork<2> >, bool)) &VesselNetworkGeometryCalculator2::GetExtents, 
            " " , py::arg("pNetwork"), py::arg("useRadii") = false )
        .def_static(
            "VesselCrossesLineSegment", 
            (bool(*)(::std::shared_ptr<VesselNetwork<2> >, ::Vertex<2> const &, ::Vertex<2> const &, double)) &VesselNetworkGeometryCalculator2::VesselCrossesLineSegment, 
            " " , py::arg("pNetwork"), py::arg("rCoord1"), py::arg("rCoord2"), py::arg("tolerance") = 9.9999999999999995E-7 )
    ;
}
