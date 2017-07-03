#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractActorGenerator.hpp"

#include "AbstractActorGenerator2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractActorGenerator<2 > AbstractActorGenerator2;
;

class AbstractActorGenerator2_Overloads : public AbstractActorGenerator2{
    public:
    using AbstractActorGenerator2::AbstractActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractActorGenerator2,
            AddActor,
            pRenderer);
    }

};
void register_AbstractActorGenerator2_class(py::module &m){
py::class_<AbstractActorGenerator2 , AbstractActorGenerator2_Overloads   >(m, "AbstractActorGenerator2")
        .def(
            "AddActor", 
            (void(AbstractActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &AbstractActorGenerator2::AddActor, 
            " " , py::arg("pRenderer"))
        .def(
            "GetColorTransferFunction", 
            (::vtkSmartPointer<vtkColorTransferFunction>(AbstractActorGenerator2::*)()) &AbstractActorGenerator2::GetColorTransferFunction, 
            " " )
        .def(
            "GetDiscreteColorTransferFunction", 
            (::vtkSmartPointer<vtkColorTransferFunction>(AbstractActorGenerator2::*)()) &AbstractActorGenerator2::GetDiscreteColorTransferFunction, 
            " " )
        .def(
            "GetScaleBar", 
            (::vtkSmartPointer<vtkScalarBarActor>(AbstractActorGenerator2::*)()) &AbstractActorGenerator2::GetScaleBar, 
            " " )
        .def(
            "SetShowEdges", 
            (void(AbstractActorGenerator2::*)(bool)) &AbstractActorGenerator2::SetShowEdges, 
            " " , py::arg("show"))
        .def(
            "SetShowPoints", 
            (void(AbstractActorGenerator2::*)(bool)) &AbstractActorGenerator2::SetShowPoints, 
            " " , py::arg("show"))
        .def(
            "SetShowVolume", 
            (void(AbstractActorGenerator2::*)(bool)) &AbstractActorGenerator2::SetShowVolume, 
            " " , py::arg("show"))
        .def(
            "SetEdgeColor", 
            (void(AbstractActorGenerator2::*)(::boost::numeric::ublas::c_vector<double, 3> const &)) &AbstractActorGenerator2::SetEdgeColor, 
            " " , py::arg("rColor"))
        .def(
            "SetPointColor", 
            (void(AbstractActorGenerator2::*)(::boost::numeric::ublas::c_vector<double, 3> const &)) &AbstractActorGenerator2::SetPointColor, 
            " " , py::arg("rColor"))
        .def(
            "SetVolumeColor", 
            (void(AbstractActorGenerator2::*)(::boost::numeric::ublas::c_vector<double, 3> const &)) &AbstractActorGenerator2::SetVolumeColor, 
            " " , py::arg("rColor"))
        .def(
            "SetVolumeOpacity", 
            (void(AbstractActorGenerator2::*)(double)) &AbstractActorGenerator2::SetVolumeOpacity, 
            " " , py::arg("opacity"))
        .def(
            "SetShowScaleBar", 
            (void(AbstractActorGenerator2::*)(double)) &AbstractActorGenerator2::SetShowScaleBar, 
            " " , py::arg("show"))
        .def(
            "SetPointSize", 
            (void(AbstractActorGenerator2::*)(double)) &AbstractActorGenerator2::SetPointSize, 
            " " , py::arg("size"))
        .def(
            "SetEdgeSize", 
            (void(AbstractActorGenerator2::*)(double)) &AbstractActorGenerator2::SetEdgeSize, 
            " " , py::arg("size"))
        .def(
            "SetDataLabel", 
            (void(AbstractActorGenerator2::*)(::std::string const &)) &AbstractActorGenerator2::SetDataLabel, 
            " " , py::arg("rLabel"))
    ;
}
