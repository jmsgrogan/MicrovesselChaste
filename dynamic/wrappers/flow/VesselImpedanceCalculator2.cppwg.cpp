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
#include "VesselImpedanceCalculator.hpp"

#include "PythonObjectConverters.hpp"
#include "VesselImpedanceCalculator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VesselImpedanceCalculator<2 > VesselImpedanceCalculator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

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
py::class_<VesselImpedanceCalculator2 , VesselImpedanceCalculator2_Overloads , std::shared_ptr<VesselImpedanceCalculator2 >  , AbstractVesselNetworkCalculator<2>  >(m, "VesselImpedanceCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselImpedanceCalculator<2> >(*)()) &VesselImpedanceCalculator2::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(VesselImpedanceCalculator2::*)()) &VesselImpedanceCalculator2::Calculate, 
            " "  )
    ;
}
