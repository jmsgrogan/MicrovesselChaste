#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "ViscosityCalculator.hpp"

#include "ViscosityCalculator3.cppwg.hpp"

namespace py = pybind11;
typedef ViscosityCalculator<3 > ViscosityCalculator3;
;

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
py::class_<ViscosityCalculator3 , ViscosityCalculator3_Overloads   >(m, "ViscosityCalculator3")
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