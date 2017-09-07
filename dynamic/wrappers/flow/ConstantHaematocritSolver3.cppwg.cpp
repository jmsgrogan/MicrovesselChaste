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
#include "ConstantHaematocritSolver.hpp"

#include "PythonObjectConverters.hpp"
#include "ConstantHaematocritSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef ConstantHaematocritSolver<3 > ConstantHaematocritSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class ConstantHaematocritSolver3_Overloads : public ConstantHaematocritSolver3{
    public:
    using ConstantHaematocritSolver3::ConstantHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ConstantHaematocritSolver3,
            Calculate,
            );
    }

};
void register_ConstantHaematocritSolver3_class(py::module &m){
py::class_<ConstantHaematocritSolver3 , ConstantHaematocritSolver3_Overloads , std::shared_ptr<ConstantHaematocritSolver3 >  , AbstractHaematocritSolver<3>  >(m, "ConstantHaematocritSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ConstantHaematocritSolver<3> >(*)()) &ConstantHaematocritSolver3::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(ConstantHaematocritSolver3::*)()) &ConstantHaematocritSolver3::Calculate, 
            " "  )
        .def(
            "SetHaematocrit", 
            (void(ConstantHaematocritSolver3::*)(::QDimensionless)) &ConstantHaematocritSolver3::SetHaematocrit, 
            " " , py::arg("haematocrit") )
    ;
}
