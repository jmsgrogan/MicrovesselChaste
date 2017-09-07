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
#include "AlarconHaematocritSolver2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AlarconHaematocritSolver<2 > AlarconHaematocritSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AlarconHaematocritSolver2_Overloads : public AlarconHaematocritSolver2{
    public:
    using AlarconHaematocritSolver2::AlarconHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            AlarconHaematocritSolver2,
            Calculate,
            );
    }

};
void register_AlarconHaematocritSolver2_class(py::module &m){
py::class_<AlarconHaematocritSolver2 , AlarconHaematocritSolver2_Overloads , std::shared_ptr<AlarconHaematocritSolver2 >  , AbstractHaematocritSolver<2>  >(m, "AlarconHaematocritSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AlarconHaematocritSolver<2> >(*)()) &AlarconHaematocritSolver2::Create, 
            " "  )
        .def(
            "Calculate", 
            (void(AlarconHaematocritSolver2::*)()) &AlarconHaematocritSolver2::Calculate, 
            " "  )
        .def(
            "SetTHR", 
            (void(AlarconHaematocritSolver2::*)(::QDimensionless)) &AlarconHaematocritSolver2::SetTHR, 
            " " , py::arg("thr") )
        .def(
            "SetAlpha", 
            (void(AlarconHaematocritSolver2::*)(::QDimensionless)) &AlarconHaematocritSolver2::SetAlpha, 
            " " , py::arg("alpha") )
        .def(
            "SetHaematocrit", 
            (void(AlarconHaematocritSolver2::*)(::QDimensionless)) &AlarconHaematocritSolver2::SetHaematocrit, 
            " " , py::arg("haematocrit") )
    ;
}
