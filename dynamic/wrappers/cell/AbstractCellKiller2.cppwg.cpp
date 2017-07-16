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
#include "AbstractCellKiller.hpp"

#include "AbstractCellKiller2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractCellKiller<2 > AbstractCellKiller2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractCellKiller2_Overloads : public AbstractCellKiller2{
    public:
    using AbstractCellKiller2::AbstractCellKiller;
    void CheckAndLabelCellsForApoptosisOrDeath() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractCellKiller2,
            CheckAndLabelCellsForApoptosisOrDeath,
            );
    }
    void OutputCellKillerParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractCellKiller2,
            OutputCellKillerParameters,
            rParamsFile);
    }

};
void register_AbstractCellKiller2_class(py::module &m){
py::class_<AbstractCellKiller2 , AbstractCellKiller2_Overloads , std::shared_ptr<AbstractCellKiller2 >   >(m, "AbstractCellKiller2")
        .def(py::init<::AbstractCellPopulation<2, 2> * >(), py::arg("pCellPopulation"))
        .def(
            "CheckAndLabelCellsForApoptosisOrDeath", 
            (void(AbstractCellKiller2::*)()) &AbstractCellKiller2::CheckAndLabelCellsForApoptosisOrDeath, 
            " "  )
        .def(
            "GetCellPopulation", 
            (::AbstractCellPopulation<2, 2> const *(AbstractCellKiller2::*)() const ) &AbstractCellKiller2::GetCellPopulation, 
            " "  , py::return_value_policy::reference)
        .def(
            "OutputCellKillerInfo", 
            (void(AbstractCellKiller2::*)(::out_stream &)) &AbstractCellKiller2::OutputCellKillerInfo, 
            " " , py::arg("rParamsFile") )
        .def(
            "OutputCellKillerParameters", 
            (void(AbstractCellKiller2::*)(::out_stream &)) &AbstractCellKiller2::OutputCellKillerParameters, 
            " " , py::arg("rParamsFile") )
    ;
}
