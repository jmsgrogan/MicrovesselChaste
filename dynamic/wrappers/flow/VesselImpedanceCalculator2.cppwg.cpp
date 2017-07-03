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

#include "VesselImpedanceCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef VesselImpedanceCalculator<2 > VesselImpedanceCalculator2;
;

class VesselImpedanceCalculator2_Overloads : public VesselImpedanceCalculator2{
    public:
    using VesselImpedanceCalculator2::VesselImpedanceCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            VesselImpedanceCalculator2,
            Calculate,
            );
    }

};
void register_VesselImpedanceCalculator2_class(py::module &m){
py::class_<VesselImpedanceCalculator2 , VesselImpedanceCalculator2_Overloads   >(m, "VesselImpedanceCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselImpedanceCalculator<2> >(*)()) &VesselImpedanceCalculator2::Create, 
            " " )
        .def(
            "Calculate", 
            (void(VesselImpedanceCalculator2::*)()) &VesselImpedanceCalculator2::Calculate, 
            " " )
    ;
}
