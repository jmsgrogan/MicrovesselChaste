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

#include "PythonObjectConverters.hpp"
#include "WallShearStressCalculator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef WallShearStressCalculator<2 > WallShearStressCalculator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

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
py::class_<WallShearStressCalculator2 , WallShearStressCalculator2_Overloads , std::shared_ptr<WallShearStressCalculator2 >  , AbstractVesselNetworkCalculator<2>  >(m, "WallShearStressCalculator2")
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
