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

#include "PythonObjectConverters.hpp"
#include "AbstractVesselNetworkCalculator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AbstractVesselNetworkCalculator<3 > AbstractVesselNetworkCalculator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractVesselNetworkCalculator3_Overloads : public AbstractVesselNetworkCalculator3{
    public:
    using AbstractVesselNetworkCalculator3::AbstractVesselNetworkCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractVesselNetworkCalculator3,
            Calculate,
            );
    }

};
void register_AbstractVesselNetworkCalculator3_class(py::module &m){
py::class_<AbstractVesselNetworkCalculator3 , AbstractVesselNetworkCalculator3_Overloads , std::shared_ptr<AbstractVesselNetworkCalculator3 >   >(m, "AbstractVesselNetworkCalculator3")
        .def(py::init< >())
        .def(
            "SetVesselNetwork", 
            (void(AbstractVesselNetworkCalculator3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AbstractVesselNetworkCalculator3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "Calculate", 
            (void(AbstractVesselNetworkCalculator3::*)()) &AbstractVesselNetworkCalculator3::Calculate, 
            " "  )
    ;
}
