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
#include "AbstractActorGenerator.hpp"

#include "AbstractActorGenerator3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractActorGenerator<3 > AbstractActorGenerator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractActorGenerator3_Overloads : public AbstractActorGenerator3{
    public:
    using AbstractActorGenerator3::AbstractActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractActorGenerator3,
            AddActor,
            pRenderer);
    }

};
void register_AbstractActorGenerator3_class(py::module &m){
py::class_<AbstractActorGenerator3 , AbstractActorGenerator3_Overloads , std::shared_ptr<AbstractActorGenerator3 >   >(m, "AbstractActorGenerator3")
        .def(
            "AddActor", 
            (void(AbstractActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &AbstractActorGenerator3::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "GetColorTransferFunction", 
            (::vtkSmartPointer<vtkColorTransferFunction>(AbstractActorGenerator3::*)()) &AbstractActorGenerator3::GetColorTransferFunction, 
            " "  )
        .def(
            "GetDiscreteColorTransferFunction", 
            (::vtkSmartPointer<vtkColorTransferFunction>(AbstractActorGenerator3::*)()) &AbstractActorGenerator3::GetDiscreteColorTransferFunction, 
            " "  )
        .def(
            "GetScaleBar", 
            (::vtkSmartPointer<vtkScalarBarActor>(AbstractActorGenerator3::*)()) &AbstractActorGenerator3::GetScaleBar, 
            " "  )
        .def(
            "SetShowEdges", 
            (void(AbstractActorGenerator3::*)(bool)) &AbstractActorGenerator3::SetShowEdges, 
            " " , py::arg("show") )
        .def(
            "SetShowPoints", 
            (void(AbstractActorGenerator3::*)(bool)) &AbstractActorGenerator3::SetShowPoints, 
            " " , py::arg("show") )
        .def(
            "SetShowVolume", 
            (void(AbstractActorGenerator3::*)(bool)) &AbstractActorGenerator3::SetShowVolume, 
            " " , py::arg("show") )
        .def(
            "SetEdgeColor", 
            (void(AbstractActorGenerator3::*)(::boost::numeric::ublas::c_vector<double, 3> const &)) &AbstractActorGenerator3::SetEdgeColor, 
            " " , py::arg("rColor") )
        .def(
            "SetPointColor", 
            (void(AbstractActorGenerator3::*)(::boost::numeric::ublas::c_vector<double, 3> const &)) &AbstractActorGenerator3::SetPointColor, 
            " " , py::arg("rColor") )
        .def(
            "SetVolumeColor", 
            (void(AbstractActorGenerator3::*)(::boost::numeric::ublas::c_vector<double, 3> const &)) &AbstractActorGenerator3::SetVolumeColor, 
            " " , py::arg("rColor") )
        .def(
            "SetVolumeOpacity", 
            (void(AbstractActorGenerator3::*)(double)) &AbstractActorGenerator3::SetVolumeOpacity, 
            " " , py::arg("opacity") )
        .def(
            "SetShowScaleBar", 
            (void(AbstractActorGenerator3::*)(double)) &AbstractActorGenerator3::SetShowScaleBar, 
            " " , py::arg("show") )
        .def(
            "SetPointSize", 
            (void(AbstractActorGenerator3::*)(double)) &AbstractActorGenerator3::SetPointSize, 
            " " , py::arg("size") )
        .def(
            "SetEdgeSize", 
            (void(AbstractActorGenerator3::*)(double)) &AbstractActorGenerator3::SetEdgeSize, 
            " " , py::arg("size") )
        .def(
            "SetDataLabel", 
            (void(AbstractActorGenerator3::*)(::std::string const &)) &AbstractActorGenerator3::SetDataLabel, 
            " " , py::arg("rLabel") )
    ;
}
