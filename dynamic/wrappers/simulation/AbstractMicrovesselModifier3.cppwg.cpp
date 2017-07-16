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
#include "AbstractMicrovesselModifier.hpp"

#include "AbstractMicrovesselModifier3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractMicrovesselModifier<3 > AbstractMicrovesselModifier3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractMicrovesselModifier3_Overloads : public AbstractMicrovesselModifier3{
    public:
    using AbstractMicrovesselModifier3::AbstractMicrovesselModifier;
    void SetupSolve(::std::string outputDirectory) override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractMicrovesselModifier3,
            SetupSolve,
            outputDirectory);
    }
    void UpdateAtEndOfTimeStep() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractMicrovesselModifier3,
            UpdateAtEndOfTimeStep,
            );
    }

};
void register_AbstractMicrovesselModifier3_class(py::module &m){
py::class_<AbstractMicrovesselModifier3 , AbstractMicrovesselModifier3_Overloads , std::shared_ptr<AbstractMicrovesselModifier3 >   >(m, "AbstractMicrovesselModifier3")
        .def(py::init< >())
        .def(
            "AddDiscreteContinuumSolver", 
            (void(AbstractMicrovesselModifier3::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<3> >)) &AbstractMicrovesselModifier3::AddDiscreteContinuumSolver, 
            " " , py::arg("pDiscreteContinuumSolver") )
        .def(
            "GetCellPopulation", 
            (::std::shared_ptr<AbstractCellPopulation<3, 3> >(AbstractMicrovesselModifier3::*)()) &AbstractMicrovesselModifier3::GetCellPopulation, 
            " "  )
        .def(
            "GetVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<3> >(AbstractMicrovesselModifier3::*)()) &AbstractMicrovesselModifier3::GetVesselNetwork, 
            " "  )
        .def(
            "GetNumberOfDiscreteContinuumSolvers", 
            (unsigned int(AbstractMicrovesselModifier3::*)()) &AbstractMicrovesselModifier3::GetNumberOfDiscreteContinuumSolvers, 
            " "  )
        .def(
            "GetDiscreteContinuumSolver", 
            (::std::shared_ptr<AbstractDiscreteContinuumSolver<3> >(AbstractMicrovesselModifier3::*)(unsigned int)) &AbstractMicrovesselModifier3::GetDiscreteContinuumSolver, 
            " " , py::arg("index") )
        .def(
            "SetCellPopulation", 
            (void(AbstractMicrovesselModifier3::*)(::std::shared_ptr<AbstractCellPopulation<3, 3> >)) &AbstractMicrovesselModifier3::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetVesselNetwork", 
            (void(AbstractMicrovesselModifier3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AbstractMicrovesselModifier3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetupSolve", 
            (void(AbstractMicrovesselModifier3::*)(::std::string)) &AbstractMicrovesselModifier3::SetupSolve, 
            " " , py::arg("outputDirectory") )
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(AbstractMicrovesselModifier3::*)()) &AbstractMicrovesselModifier3::UpdateAtEndOfTimeStep, 
            " "  )
    ;
}
