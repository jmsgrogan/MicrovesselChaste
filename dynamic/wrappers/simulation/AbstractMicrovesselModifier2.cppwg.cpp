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

#include "PythonObjectConverters.hpp"
#include "AbstractMicrovesselModifier2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractMicrovesselModifier<2 > AbstractMicrovesselModifier2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractMicrovesselModifier2_Overloads : public AbstractMicrovesselModifier2{
    public:
    using AbstractMicrovesselModifier2::AbstractMicrovesselModifier;
    void SetupSolve(::std::string outputDirectory) override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractMicrovesselModifier2,
            SetupSolve,
            outputDirectory);
    }
    void UpdateAtEndOfTimeStep() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractMicrovesselModifier2,
            UpdateAtEndOfTimeStep,
            );
    }

};
void register_AbstractMicrovesselModifier2_class(py::module &m){
py::class_<AbstractMicrovesselModifier2 , AbstractMicrovesselModifier2_Overloads , std::shared_ptr<AbstractMicrovesselModifier2 >   >(m, "AbstractMicrovesselModifier2")
        .def(py::init< >())
        .def(
            "AddDiscreteContinuumSolver", 
            (void(AbstractMicrovesselModifier2::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<2> >)) &AbstractMicrovesselModifier2::AddDiscreteContinuumSolver, 
            " " , py::arg("pDiscreteContinuumSolver") )
        .def(
            "GetCellPopulation", 
            (::std::shared_ptr<AbstractCellPopulation<2, 2> >(AbstractMicrovesselModifier2::*)()) &AbstractMicrovesselModifier2::GetCellPopulation, 
            " "  )
        .def(
            "GetVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<2> >(AbstractMicrovesselModifier2::*)()) &AbstractMicrovesselModifier2::GetVesselNetwork, 
            " "  )
        .def(
            "GetNumberOfDiscreteContinuumSolvers", 
            (unsigned int(AbstractMicrovesselModifier2::*)()) &AbstractMicrovesselModifier2::GetNumberOfDiscreteContinuumSolvers, 
            " "  )
        .def(
            "GetDiscreteContinuumSolver", 
            (::std::shared_ptr<AbstractDiscreteContinuumSolver<2> >(AbstractMicrovesselModifier2::*)(unsigned int)) &AbstractMicrovesselModifier2::GetDiscreteContinuumSolver, 
            " " , py::arg("index") )
        .def(
            "SetCellPopulation", 
            (void(AbstractMicrovesselModifier2::*)(::std::shared_ptr<AbstractCellPopulation<2, 2> >)) &AbstractMicrovesselModifier2::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetVesselNetwork", 
            (void(AbstractMicrovesselModifier2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AbstractMicrovesselModifier2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetupSolve", 
            (void(AbstractMicrovesselModifier2::*)(::std::string)) &AbstractMicrovesselModifier2::SetupSolve, 
            " " , py::arg("outputDirectory") )
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(AbstractMicrovesselModifier2::*)()) &AbstractMicrovesselModifier2::UpdateAtEndOfTimeStep, 
            " "  )
    ;
}
