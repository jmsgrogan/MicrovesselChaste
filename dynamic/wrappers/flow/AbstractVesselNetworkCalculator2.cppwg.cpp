#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkCalculator.hpp"

#include "AbstractVesselNetworkCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkCalculator<2 > AbstractVesselNetworkCalculator2;
;

class AbstractVesselNetworkCalculator2_Overloads : public AbstractVesselNetworkCalculator2{
    public:
    using AbstractVesselNetworkCalculator2::AbstractVesselNetworkCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractVesselNetworkCalculator2,
            Calculate,
            );
    }

};
void register_AbstractVesselNetworkCalculator2_class(py::module &m){
py::class_<AbstractVesselNetworkCalculator2 , AbstractVesselNetworkCalculator2_Overloads   >(m, "AbstractVesselNetworkCalculator2")
        .def(
            "SetVesselNetwork", 
            (void(AbstractVesselNetworkCalculator2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AbstractVesselNetworkCalculator2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "Calculate", 
            (void(AbstractVesselNetworkCalculator2::*)()) &AbstractVesselNetworkCalculator2::Calculate, 
            " "  )
    ;
}
