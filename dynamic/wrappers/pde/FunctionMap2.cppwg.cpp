#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "FunctionMap.hpp"

#include "FunctionMap2.cppwg.hpp"

namespace py = pybind11;
typedef FunctionMap<2 > FunctionMap2;
;

class FunctionMap2_Overloads : public FunctionMap2{
    public:
    using FunctionMap2::FunctionMap;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            FunctionMap2,
            Solve,
            );
    }

};
void register_FunctionMap2_class(py::module &m){
py::class_<FunctionMap2 , FunctionMap2_Overloads   >(m, "FunctionMap2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<FunctionMap<2> >(*)()) &FunctionMap2::Create, 
            " "  )
        .def(
            "Solve", 
            (void(FunctionMap2::*)()) &FunctionMap2::Solve, 
            " "  )
    ;
}
