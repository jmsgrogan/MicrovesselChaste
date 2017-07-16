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

#include "ConstantHaematocritSolver2.cppwg.hpp"

namespace py = pybind11;
typedef ConstantHaematocritSolver<2 > ConstantHaematocritSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class ConstantHaematocritSolver2_Overloads : public ConstantHaematocritSolver2{
    public:
    using ConstantHaematocritSolver2::ConstantHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ConstantHaematocritSolver2,
            Calculate,
            );
    }

};
void register_ConstantHaematocritSolver2_class(py::module &m){
py::class_<ConstantHaematocritSolver2 , ConstantHaematocritSolver2_Overloads , std::shared_ptr<ConstantHaematocritSolver2 >  , AbstractHaematocritSolver<2>  >(m, "ConstantHaematocritSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ConstantHaematocritSolver<2> >(*)()) &ConstantHaematocritSolver2::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(ConstantHaematocritSolver2::*)()) &ConstantHaematocritSolver2::Calculate, 
            " "  )
        .def(
            "SetHaematocrit", 
            (void(ConstantHaematocritSolver2::*)(::QDimensionless)) &ConstantHaematocritSolver2::SetHaematocrit, 
            " " , py::arg("haematocrit") )
    ;
}
