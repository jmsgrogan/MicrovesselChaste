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

#include "DistanceMap2.cppwg.hpp"

namespace py = pybind11;
typedef DistanceMap<2 > DistanceMap2;
;

class DistanceMap2_Overloads : public DistanceMap2{
    public:
    using DistanceMap2::DistanceMap;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            DistanceMap2,
            Solve,
            );
    }

};
void register_DistanceMap2_class(py::module &m){
py::class_<DistanceMap2 , DistanceMap2_Overloads   >(m, "DistanceMap2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DistanceMap<2> >(*)()) &DistanceMap2::Create, 
            " "  )
        .def(
            "SetUseSegmentRadii", 
            (void(DistanceMap2::*)(bool)) &DistanceMap2::SetUseSegmentRadii, 
            " " , py::arg("useRadii") )
        .def(
            "Solve", 
            (void(DistanceMap2::*)()) &DistanceMap2::Solve, 
            " "  )
    ;
}
