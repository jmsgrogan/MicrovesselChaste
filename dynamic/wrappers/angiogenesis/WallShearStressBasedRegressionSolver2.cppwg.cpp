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

#include "PythonObjectConverters.hpp"
#include "WallShearStressBasedRegressionSolver2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef WallShearStressBasedRegressionSolver<2 > WallShearStressBasedRegressionSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class WallShearStressBasedRegressionSolver2_Overloads : public WallShearStressBasedRegressionSolver2{
    public:
    using WallShearStressBasedRegressionSolver2::WallShearStressBasedRegressionSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            WallShearStressBasedRegressionSolver2,
            Increment,
            );
    }

};
void register_WallShearStressBasedRegressionSolver2_class(py::module &m){
py::class_<WallShearStressBasedRegressionSolver2 , WallShearStressBasedRegressionSolver2_Overloads , std::shared_ptr<WallShearStressBasedRegressionSolver2 >  , RegressionSolver<2>  >(m, "WallShearStressBasedRegressionSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<WallShearStressBasedRegressionSolver<2> >(*)()) &WallShearStressBasedRegressionSolver2::Create, 
            " "  )
        .def(
            "GetLowWallShearStressThreshold", 
            (::QPressure(WallShearStressBasedRegressionSolver2::*)()) &WallShearStressBasedRegressionSolver2::GetLowWallShearStressThreshold, 
            " "  )
        .def(
            "GetMaximumTimeWithLowWallShearStress", 
            (::QTime(WallShearStressBasedRegressionSolver2::*)()) &WallShearStressBasedRegressionSolver2::GetMaximumTimeWithLowWallShearStress, 
            " "  )
        .def(
            "SetMaximumTimeWithLowWallShearStress", 
            (void(WallShearStressBasedRegressionSolver2::*)(::QTime)) &WallShearStressBasedRegressionSolver2::SetMaximumTimeWithLowWallShearStress, 
            " " , py::arg("time") )
        .def(
            "SetLowWallShearStressThreshold", 
            (void(WallShearStressBasedRegressionSolver2::*)(::QPressure)) &WallShearStressBasedRegressionSolver2::SetLowWallShearStressThreshold, 
            " " , py::arg("threshold") )
        .def(
            "Increment", 
            (void(WallShearStressBasedRegressionSolver2::*)()) &WallShearStressBasedRegressionSolver2::Increment, 
            " "  )
    ;
}
