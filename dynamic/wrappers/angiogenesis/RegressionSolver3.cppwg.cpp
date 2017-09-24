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
#include "RegressionSolver.hpp"

#include "PythonObjectConverters.hpp"
#include "RegressionSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef RegressionSolver<3 > RegressionSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class RegressionSolver3_Overloads : public RegressionSolver3{
    public:
    using RegressionSolver3::RegressionSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            RegressionSolver3,
            Increment,
            );
    }

};
void register_RegressionSolver3_class(py::module &m){
py::class_<RegressionSolver3 , RegressionSolver3_Overloads , std::shared_ptr<RegressionSolver3 >   >(m, "RegressionSolver3")
        .def(py::init< >())
        .def(
            "SetVesselNetwork", 
            (void(RegressionSolver3::*)(::std::shared_ptr<VesselNetwork<3> >)) &RegressionSolver3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "Increment", 
            (void(RegressionSolver3::*)()) &RegressionSolver3::Increment, 
            " "  )
    ;
}
