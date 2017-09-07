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
#include "ViscosityCalculator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef ViscosityCalculator<2 > ViscosityCalculator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class ViscosityCalculator2_Overloads : public ViscosityCalculator2{
    public:
    using ViscosityCalculator2::ViscosityCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ViscosityCalculator2,
            Calculate,
            );
    }

};
void register_ViscosityCalculator2_class(py::module &m){
py::class_<ViscosityCalculator2 , ViscosityCalculator2_Overloads , std::shared_ptr<ViscosityCalculator2 >  , AbstractVesselNetworkCalculator<2>  >(m, "ViscosityCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ViscosityCalculator<2> >(*)()) &ViscosityCalculator2::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(ViscosityCalculator2::*)()) &ViscosityCalculator2::Calculate, 
            " "  )
        .def(
            "SetPlasmaViscosity", 
            (void(ViscosityCalculator2::*)(::QDynamicViscosity)) &ViscosityCalculator2::SetPlasmaViscosity, 
            " " , py::arg("viscosity") )
    ;
}
