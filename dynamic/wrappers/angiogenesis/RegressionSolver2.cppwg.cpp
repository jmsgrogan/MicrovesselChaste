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

#include "RegressionSolver2.cppwg.hpp"

namespace py = pybind11;
typedef RegressionSolver<2 > RegressionSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class RegressionSolver2_Overloads : public RegressionSolver2{
    public:
    using RegressionSolver2::RegressionSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            RegressionSolver2,
            Increment,
            );
    }

};
void register_RegressionSolver2_class(py::module &m){
py::class_<RegressionSolver2 , RegressionSolver2_Overloads , std::shared_ptr<RegressionSolver2 >   >(m, "RegressionSolver2")
        .def(py::init< >())
        .def(
            "SetVesselNetwork", 
            (void(RegressionSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &RegressionSolver2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "Increment", 
            (void(RegressionSolver2::*)()) &RegressionSolver2::Increment, 
            " "  )
    ;
}
