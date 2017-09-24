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

#include "PythonObjectConverters.hpp"
#include "RadiusCalculator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef RadiusCalculator<3 > RadiusCalculator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class RadiusCalculator3_Overloads : public RadiusCalculator3{
    public:
    using RadiusCalculator3::RadiusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            RadiusCalculator3,
            Calculate,
            );
    }

};
void register_RadiusCalculator3_class(py::module &m){
py::class_<RadiusCalculator3 , RadiusCalculator3_Overloads , std::shared_ptr<RadiusCalculator3 >  , AbstractVesselNetworkCalculator<3>  >(m, "RadiusCalculator3")
        .def(py::init< >())
        .def(
            "SetMinRadius", 
            (void(RadiusCalculator3::*)(::QLength)) &RadiusCalculator3::SetMinRadius, 
            " " , py::arg("minRadius") )
        .def(
            "SetMaxRadius", 
            (void(RadiusCalculator3::*)(::QLength)) &RadiusCalculator3::SetMaxRadius, 
            " " , py::arg("maxRadius") )
        .def(
            "SetTimestep", 
            (void(RadiusCalculator3::*)(::QTime)) &RadiusCalculator3::SetTimestep, 
            " " , py::arg("dt") )
        .def(
            "Calculate", 
            (void(RadiusCalculator3::*)()) &RadiusCalculator3::Calculate, 
            " "  )
    ;
}
