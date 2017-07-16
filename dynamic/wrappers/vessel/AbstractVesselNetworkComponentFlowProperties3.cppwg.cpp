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

#include "AbstractVesselNetworkComponentFlowProperties3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponentFlowProperties<3 > AbstractVesselNetworkComponentFlowProperties3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractVesselNetworkComponentFlowProperties3_Overloads : public AbstractVesselNetworkComponentFlowProperties3{
    public:
    using AbstractVesselNetworkComponentFlowProperties3::AbstractVesselNetworkComponentFlowProperties;
    void SetPressure(::QPressure pressure) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentFlowProperties3,
            SetPressure,
            pressure);
    }

};
void register_AbstractVesselNetworkComponentFlowProperties3_class(py::module &m){
py::class_<AbstractVesselNetworkComponentFlowProperties3 , AbstractVesselNetworkComponentFlowProperties3_Overloads , std::shared_ptr<AbstractVesselNetworkComponentFlowProperties3 >  , AbstractVesselNetworkComponentProperties<3>  >(m, "AbstractVesselNetworkComponentFlowProperties3")
        .def(
            "GetPressure", 
            (::QPressure(AbstractVesselNetworkComponentFlowProperties3::*)() const ) &AbstractVesselNetworkComponentFlowProperties3::GetPressure, 
            " "  )
        .def(
            "SetPressure", 
            (void(AbstractVesselNetworkComponentFlowProperties3::*)(::QPressure)) &AbstractVesselNetworkComponentFlowProperties3::SetPressure, 
            " " , py::arg("pressure") )
    ;
}
