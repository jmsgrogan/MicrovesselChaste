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
#include "AbstractVesselNetworkComponentFlowProperties.hpp"

#include "AbstractVesselNetworkComponentFlowProperties2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponentFlowProperties<2 > AbstractVesselNetworkComponentFlowProperties2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractVesselNetworkComponentFlowProperties2_Overloads : public AbstractVesselNetworkComponentFlowProperties2{
    public:
    using AbstractVesselNetworkComponentFlowProperties2::AbstractVesselNetworkComponentFlowProperties;
    void SetPressure(::QPressure pressure) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentFlowProperties2,
            SetPressure,
            pressure);
    }

};
void register_AbstractVesselNetworkComponentFlowProperties2_class(py::module &m){
py::class_<AbstractVesselNetworkComponentFlowProperties2 , AbstractVesselNetworkComponentFlowProperties2_Overloads , std::shared_ptr<AbstractVesselNetworkComponentFlowProperties2 >   >(m, "AbstractVesselNetworkComponentFlowProperties2")
        .def(
            "GetPressure", 
            (::QPressure(AbstractVesselNetworkComponentFlowProperties2::*)() const ) &AbstractVesselNetworkComponentFlowProperties2::GetPressure, 
            " "  )
        .def(
            "SetPressure", 
            (void(AbstractVesselNetworkComponentFlowProperties2::*)(::QPressure)) &AbstractVesselNetworkComponentFlowProperties2::SetPressure, 
            " " , py::arg("pressure") )
    ;
}
