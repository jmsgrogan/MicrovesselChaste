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
#include "ViscosityCalculator.hpp"

#include "PythonObjectConverters.hpp"
#include "ViscosityCalculator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef ViscosityCalculator<3 > ViscosityCalculator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class ViscosityCalculator3_Overloads : public ViscosityCalculator3{
    public:
    using ViscosityCalculator3::ViscosityCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ViscosityCalculator3,
            Calculate,
            );
    }

};
void register_ViscosityCalculator3_class(py::module &m){
py::class_<ViscosityCalculator3 , ViscosityCalculator3_Overloads , std::shared_ptr<ViscosityCalculator3 >  , AbstractVesselNetworkCalculator<3>  >(m, "ViscosityCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ViscosityCalculator<3> >(*)()) &ViscosityCalculator3::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(ViscosityCalculator3::*)()) &ViscosityCalculator3::Calculate, 
            " "  )
        .def(
            "SetPlasmaViscosity", 
            (void(ViscosityCalculator3::*)(::QDynamicViscosity)) &ViscosityCalculator3::SetPlasmaViscosity, 
            " " , py::arg("viscosity") )
    ;
}
