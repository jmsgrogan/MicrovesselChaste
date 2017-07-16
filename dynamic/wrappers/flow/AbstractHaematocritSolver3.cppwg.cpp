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
#include "AbstractHaematocritSolver.hpp"

#include "AbstractHaematocritSolver3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractHaematocritSolver<3 > AbstractHaematocritSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractHaematocritSolver3_Overloads : public AbstractHaematocritSolver3{
    public:
    using AbstractHaematocritSolver3::AbstractHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractHaematocritSolver3,
            Calculate,
            );
    }

};
void register_AbstractHaematocritSolver3_class(py::module &m){
py::class_<AbstractHaematocritSolver3 , AbstractHaematocritSolver3_Overloads , std::shared_ptr<AbstractHaematocritSolver3 >  , AbstractVesselNetworkCalculator<3>  >(m, "AbstractHaematocritSolver3")
        .def(
            "Calculate", 
            (void(AbstractHaematocritSolver3::*)()) &AbstractHaematocritSolver3::Calculate, 
            " "  )
    ;
}
