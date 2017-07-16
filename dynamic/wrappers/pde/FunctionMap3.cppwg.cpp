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
#include "FunctionMap.hpp"

#include "FunctionMap3.cppwg.hpp"

namespace py = pybind11;
typedef FunctionMap<3 > FunctionMap3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class FunctionMap3_Overloads : public FunctionMap3{
    public:
    using FunctionMap3::FunctionMap;
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            FunctionMap3,
            Solve,
            );
    }

};
void register_FunctionMap3_class(py::module &m){
py::class_<FunctionMap3 , FunctionMap3_Overloads , std::shared_ptr<FunctionMap3 >  , AbstractMixedGridDiscreteContinuumSolver<3>  >(m, "FunctionMap3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<FunctionMap<3> >(*)()) &FunctionMap3::Create, 
            " "  )
        .def(
            "Solve", 
            (void(FunctionMap3::*)()) &FunctionMap3::Solve, 
            " "  )
    ;
}
