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
#include "AlarconHaematocritSolver.hpp"

#include "PythonObjectConverters.hpp"
#include "AlarconHaematocritSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AlarconHaematocritSolver<3 > AlarconHaematocritSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AlarconHaematocritSolver3_Overloads : public AlarconHaematocritSolver3{
    public:
    using AlarconHaematocritSolver3::AlarconHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            AlarconHaematocritSolver3,
            Calculate,
            );
    }

};
void register_AlarconHaematocritSolver3_class(py::module &m){
py::class_<AlarconHaematocritSolver3 , AlarconHaematocritSolver3_Overloads , std::shared_ptr<AlarconHaematocritSolver3 >  , AbstractHaematocritSolver<3>  >(m, "AlarconHaematocritSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AlarconHaematocritSolver<3> >(*)()) &AlarconHaematocritSolver3::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(AlarconHaematocritSolver3::*)()) &AlarconHaematocritSolver3::Calculate, 
            " "  )
        .def(
            "SetTHR", 
            (void(AlarconHaematocritSolver3::*)(::QDimensionless)) &AlarconHaematocritSolver3::SetTHR, 
            " " , py::arg("thr") )
        .def(
            "SetAlpha", 
            (void(AlarconHaematocritSolver3::*)(::QDimensionless)) &AlarconHaematocritSolver3::SetAlpha, 
            " " , py::arg("alpha") )
        .def(
            "SetHaematocrit", 
            (void(AlarconHaematocritSolver3::*)(::QDimensionless)) &AlarconHaematocritSolver3::SetHaematocrit, 
            " " , py::arg("haematocrit") )
    ;
}
