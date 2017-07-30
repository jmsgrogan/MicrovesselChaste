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
#include "AbstractVesselNetworkComponentCellularProperties.hpp"

#include "AbstractVesselNetworkComponentCellularProperties3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponentCellularProperties<3 > AbstractVesselNetworkComponentCellularProperties3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractVesselNetworkComponentCellularProperties3_Overloads : public AbstractVesselNetworkComponentCellularProperties3{
    public:
    using AbstractVesselNetworkComponentCellularProperties3::AbstractVesselNetworkComponentCellularProperties;
    void SetAverageCellLengthLongitudinal(::QLength cellLength) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentCellularProperties3,
            SetAverageCellLengthLongitudinal,
            cellLength);
    }
    void SetAverageCellLengthCircumferential(::QLength cellLength) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentCellularProperties3,
            SetAverageCellLengthCircumferential,
            cellLength);
    }

};
void register_AbstractVesselNetworkComponentCellularProperties3_class(py::module &m){
py::class_<AbstractVesselNetworkComponentCellularProperties3 , AbstractVesselNetworkComponentCellularProperties3_Overloads , std::shared_ptr<AbstractVesselNetworkComponentCellularProperties3 >  , AbstractVesselNetworkComponentProperties<3>  >(m, "AbstractVesselNetworkComponentCellularProperties3")
        .def(
            "GetAverageCellLengthLongitudinal", 
            (::QLength(AbstractVesselNetworkComponentCellularProperties3::*)() const ) &AbstractVesselNetworkComponentCellularProperties3::GetAverageCellLengthLongitudinal, 
            " "  )
        .def(
            "GetAverageCellLengthCircumferential", 
            (::QLength(AbstractVesselNetworkComponentCellularProperties3::*)() const ) &AbstractVesselNetworkComponentCellularProperties3::GetAverageCellLengthCircumferential, 
            " "  )
        .def(
            "SetAverageCellLengthLongitudinal", 
            (void(AbstractVesselNetworkComponentCellularProperties3::*)(::QLength)) &AbstractVesselNetworkComponentCellularProperties3::SetAverageCellLengthLongitudinal, 
            " " , py::arg("cellLength") )
        .def(
            "SetAverageCellLengthCircumferential", 
            (void(AbstractVesselNetworkComponentCellularProperties3::*)(::QLength)) &AbstractVesselNetworkComponentCellularProperties3::SetAverageCellLengthCircumferential, 
            " " , py::arg("cellLength") )
    ;
}
