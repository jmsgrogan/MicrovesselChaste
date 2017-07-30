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

#include "AbstractVesselNetworkComponentCellularProperties2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponentCellularProperties<2 > AbstractVesselNetworkComponentCellularProperties2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractVesselNetworkComponentCellularProperties2_Overloads : public AbstractVesselNetworkComponentCellularProperties2{
    public:
    using AbstractVesselNetworkComponentCellularProperties2::AbstractVesselNetworkComponentCellularProperties;
    void SetAverageCellLengthLongitudinal(::QLength cellLength) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentCellularProperties2,
            SetAverageCellLengthLongitudinal,
            cellLength);
    }
    void SetAverageCellLengthCircumferential(::QLength cellLength) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponentCellularProperties2,
            SetAverageCellLengthCircumferential,
            cellLength);
    }

};
void register_AbstractVesselNetworkComponentCellularProperties2_class(py::module &m){
py::class_<AbstractVesselNetworkComponentCellularProperties2 , AbstractVesselNetworkComponentCellularProperties2_Overloads , std::shared_ptr<AbstractVesselNetworkComponentCellularProperties2 >  , AbstractVesselNetworkComponentProperties<2>  >(m, "AbstractVesselNetworkComponentCellularProperties2")
        .def(
            "GetAverageCellLengthLongitudinal", 
            (::QLength(AbstractVesselNetworkComponentCellularProperties2::*)() const ) &AbstractVesselNetworkComponentCellularProperties2::GetAverageCellLengthLongitudinal, 
            " "  )
        .def(
            "GetAverageCellLengthCircumferential", 
            (::QLength(AbstractVesselNetworkComponentCellularProperties2::*)() const ) &AbstractVesselNetworkComponentCellularProperties2::GetAverageCellLengthCircumferential, 
            " "  )
        .def(
            "SetAverageCellLengthLongitudinal", 
            (void(AbstractVesselNetworkComponentCellularProperties2::*)(::QLength)) &AbstractVesselNetworkComponentCellularProperties2::SetAverageCellLengthLongitudinal, 
            " " , py::arg("cellLength") )
        .def(
            "SetAverageCellLengthCircumferential", 
            (void(AbstractVesselNetworkComponentCellularProperties2::*)(::QLength)) &AbstractVesselNetworkComponentCellularProperties2::SetAverageCellLengthCircumferential, 
            " " , py::arg("cellLength") )
    ;
}
