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
#include "WallShearStressBasedRegressionSolver.hpp"

#include "WallShearStressBasedRegressionSolver3.cppwg.hpp"

namespace py = pybind11;
typedef WallShearStressBasedRegressionSolver<3 > WallShearStressBasedRegressionSolver3;
;

class WallShearStressBasedRegressionSolver3_Overloads : public WallShearStressBasedRegressionSolver3{
    public:
    using WallShearStressBasedRegressionSolver3::WallShearStressBasedRegressionSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            WallShearStressBasedRegressionSolver3,
            Increment,
            );
    }

};
void register_WallShearStressBasedRegressionSolver3_class(py::module &m){
py::class_<WallShearStressBasedRegressionSolver3 , WallShearStressBasedRegressionSolver3_Overloads   >(m, "WallShearStressBasedRegressionSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<WallShearStressBasedRegressionSolver<3> >(*)()) &WallShearStressBasedRegressionSolver3::Create, 
            " "  )
        .def(
            "GetLowWallShearStressThreshold", 
            (::QPressure(WallShearStressBasedRegressionSolver3::*)()) &WallShearStressBasedRegressionSolver3::GetLowWallShearStressThreshold, 
            " "  )
        .def(
            "GetMaximumTimeWithLowWallShearStress", 
            (::QTime(WallShearStressBasedRegressionSolver3::*)()) &WallShearStressBasedRegressionSolver3::GetMaximumTimeWithLowWallShearStress, 
            " "  )
        .def(
            "SetMaximumTimeWithLowWallShearStress", 
            (void(WallShearStressBasedRegressionSolver3::*)(::QTime)) &WallShearStressBasedRegressionSolver3::SetMaximumTimeWithLowWallShearStress, 
            " " , py::arg("time") )
        .def(
            "SetLowWallShearStressThreshold", 
            (void(WallShearStressBasedRegressionSolver3::*)(::QPressure)) &WallShearStressBasedRegressionSolver3::SetLowWallShearStressThreshold, 
            " " , py::arg("threshold") )
        .def(
            "Increment", 
            (void(WallShearStressBasedRegressionSolver3::*)()) &WallShearStressBasedRegressionSolver3::Increment, 
            " "  )
    ;
}
