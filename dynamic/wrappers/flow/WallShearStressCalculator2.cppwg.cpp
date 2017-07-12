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
#include "WallShearStressCalculator.hpp"

#include "WallShearStressCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef WallShearStressCalculator<2 > WallShearStressCalculator2;
;

class WallShearStressCalculator2_Overloads : public WallShearStressCalculator2{
    public:
    using WallShearStressCalculator2::WallShearStressCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            WallShearStressCalculator2,
            Calculate,
            );
    }

};
void register_WallShearStressCalculator2_class(py::module &m){
py::class_<WallShearStressCalculator2 , WallShearStressCalculator2_Overloads   >(m, "WallShearStressCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<WallShearStressCalculator<2> >(*)()) &WallShearStressCalculator2::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(WallShearStressCalculator2::*)()) &WallShearStressCalculator2::Calculate, 
            " "  )
    ;
}
