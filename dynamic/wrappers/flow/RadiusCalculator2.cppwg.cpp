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
#include "RadiusCalculator.hpp"

#include "RadiusCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef RadiusCalculator<2 > RadiusCalculator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class RadiusCalculator2_Overloads : public RadiusCalculator2{
    public:
    using RadiusCalculator2::RadiusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            RadiusCalculator2,
            Calculate,
            );
    }

};
void register_RadiusCalculator2_class(py::module &m){
py::class_<RadiusCalculator2 , RadiusCalculator2_Overloads , std::shared_ptr<RadiusCalculator2 >  , AbstractVesselNetworkCalculator<2>  >(m, "RadiusCalculator2")
        .def(py::init< >())
        .def(
            "SetMinRadius", 
            (void(RadiusCalculator2::*)(::QLength)) &RadiusCalculator2::SetMinRadius, 
            " " , py::arg("minRadius") )
        .def(
            "SetMaxRadius", 
            (void(RadiusCalculator2::*)(::QLength)) &RadiusCalculator2::SetMaxRadius, 
            " " , py::arg("maxRadius") )
        .def(
            "SetTimestep", 
            (void(RadiusCalculator2::*)(::QTime)) &RadiusCalculator2::SetTimestep, 
            " " , py::arg("dt") )
        .def(
            "Calculate", 
            (void(RadiusCalculator2::*)()) &RadiusCalculator2::Calculate, 
            " "  )
    ;
}
