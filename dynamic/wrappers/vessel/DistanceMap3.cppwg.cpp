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
#include "DistanceMap.hpp"

#include "PythonObjectConverters.hpp"
#include "DistanceMap3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef DistanceMap<3 > DistanceMap3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

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
py::class_<DistanceMap3 , DistanceMap3_Overloads , std::shared_ptr<DistanceMap3 >   >(m, "DistanceMap3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DistanceMap<3> >(*)()) &DistanceMap3::Create, 
            " "  )
        .def(
            "SetUseSegmentRadii", 
            (void(DistanceMap3::*)(bool)) &DistanceMap3::SetUseSegmentRadii, 
            " " , py::arg("useRadii") )
        .def(
            "Solve", 
            (void(DistanceMap3::*)()) &DistanceMap3::Solve, 
            " "  )
    ;
}
