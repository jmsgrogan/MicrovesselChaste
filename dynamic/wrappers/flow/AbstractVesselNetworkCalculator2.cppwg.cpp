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
#include "AbstractVesselNetworkCalculator.hpp"

#include "AbstractVesselNetworkCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkCalculator<2 > AbstractVesselNetworkCalculator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

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
py::class_<AbstractVesselNetworkCalculator2 , AbstractVesselNetworkCalculator2_Overloads , std::shared_ptr<AbstractVesselNetworkCalculator2 >   >(m, "AbstractVesselNetworkCalculator2")
        .def(py::init< >())
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
