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

#include "AbstractHaematocritSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractHaematocritSolver<2 > AbstractHaematocritSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractHaematocritSolver2_Overloads : public AbstractHaematocritSolver2{
    public:
    using AbstractHaematocritSolver2::AbstractHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractHaematocritSolver2,
            Calculate,
            );
    }

};
void register_AbstractHaematocritSolver2_class(py::module &m){
py::class_<AbstractHaematocritSolver2 , AbstractHaematocritSolver2_Overloads , std::shared_ptr<AbstractHaematocritSolver2 >  , AbstractVesselNetworkCalculator<2>  >(m, "AbstractHaematocritSolver2")
        .def(
            "Calculate", 
            (void(AbstractHaematocritSolver2::*)()) &AbstractHaematocritSolver2::Calculate, 
            " "  )
    ;
}
