#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DistanceMap.hpp"

#include "DistanceMap3.cppwg.hpp"

namespace py = pybind11;
typedef DistanceMap<3 > DistanceMap3;
;

class DistanceMap3_Overloads : public DistanceMap3{
    public:
    using DistanceMap3::DistanceMap;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            DistanceMap3,
            Solve,
            );
    }

};
void register_DistanceMap3_class(py::module &m){
py::class_<DistanceMap3 , DistanceMap3_Overloads   >(m, "DistanceMap3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DistanceMap<3> >(*)()) &DistanceMap3::Create, 
            " " )
        .def(
            "SetUseSegmentRadii", 
            (void(DistanceMap3::*)(bool)) &DistanceMap3::SetUseSegmentRadii, 
            " " , py::arg("useRadii"))
        .def(
            "Solve", 
            (void(DistanceMap3::*)()) &DistanceMap3::Solve, 
            " " )
    ;
}
