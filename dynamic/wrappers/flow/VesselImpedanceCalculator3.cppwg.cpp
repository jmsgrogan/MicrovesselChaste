#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselImpedanceCalculator.hpp"

#include "VesselImpedanceCalculator3.cppwg.hpp"

namespace py = pybind11;
typedef VesselImpedanceCalculator<3 > VesselImpedanceCalculator3;
;

class VesselImpedanceCalculator3_Overloads : public VesselImpedanceCalculator3{
    public:
    using VesselImpedanceCalculator3::VesselImpedanceCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            VesselImpedanceCalculator3,
            Calculate,
            );
    }

};
void register_VesselImpedanceCalculator3_class(py::module &m){
py::class_<VesselImpedanceCalculator3 , VesselImpedanceCalculator3_Overloads   >(m, "VesselImpedanceCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselImpedanceCalculator<3> >(*)()) &VesselImpedanceCalculator3::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(VesselImpedanceCalculator3::*)()) &VesselImpedanceCalculator3::Calculate, 
            " "  )
    ;
}
