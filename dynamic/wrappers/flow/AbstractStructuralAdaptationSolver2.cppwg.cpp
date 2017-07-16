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
#include "AbstractStructuralAdaptationSolver.hpp"

#include "AbstractStructuralAdaptationSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractStructuralAdaptationSolver<2 > AbstractStructuralAdaptationSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractStructuralAdaptationSolver2_Overloads : public AbstractStructuralAdaptationSolver2{
    public:
    using AbstractStructuralAdaptationSolver2::AbstractStructuralAdaptationSolver;
    void Iterate() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractStructuralAdaptationSolver2,
            Iterate,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractStructuralAdaptationSolver2,
            Write,
            );
    }

};
void register_AbstractStructuralAdaptationSolver2_class(py::module &m){
py::class_<AbstractStructuralAdaptationSolver2 , AbstractStructuralAdaptationSolver2_Overloads , std::shared_ptr<AbstractStructuralAdaptationSolver2 >   >(m, "AbstractStructuralAdaptationSolver2")
        .def(py::init< >())
        .def(
            "GetTolerance", 
            (double(AbstractStructuralAdaptationSolver2::*)() const ) &AbstractStructuralAdaptationSolver2::GetTolerance, 
            " "  )
        .def(
            "GetWriteOutput", 
            (bool(AbstractStructuralAdaptationSolver2::*)() const ) &AbstractStructuralAdaptationSolver2::GetWriteOutput, 
            " "  )
        .def(
            "GetOutputFileName", 
            (::std::string(AbstractStructuralAdaptationSolver2::*)() const ) &AbstractStructuralAdaptationSolver2::GetOutputFileName, 
            " "  )
        .def(
            "GetTimeIncrement", 
            (::QTime(AbstractStructuralAdaptationSolver2::*)() const ) &AbstractStructuralAdaptationSolver2::GetTimeIncrement, 
            " "  )
        .def(
            "Iterate", 
            (void(AbstractStructuralAdaptationSolver2::*)()) &AbstractStructuralAdaptationSolver2::Iterate, 
            " "  )
        .def(
            "SetTolerance", 
            (void(AbstractStructuralAdaptationSolver2::*)(double)) &AbstractStructuralAdaptationSolver2::SetTolerance, 
            " " , py::arg("tolerance") )
        .def(
            "SetTimeIncrement", 
            (void(AbstractStructuralAdaptationSolver2::*)(::QTime)) &AbstractStructuralAdaptationSolver2::SetTimeIncrement, 
            " " , py::arg("timeIncrement") )
        .def(
            "SetMaxIterations", 
            (void(AbstractStructuralAdaptationSolver2::*)(unsigned int)) &AbstractStructuralAdaptationSolver2::SetMaxIterations, 
            " " , py::arg("iterations") )
        .def(
            "SetWriteOutput", 
            (void(AbstractStructuralAdaptationSolver2::*)(bool)) &AbstractStructuralAdaptationSolver2::SetWriteOutput, 
            " " , py::arg("writeFlag") )
        .def(
            "SetOutputFileName", 
            (void(AbstractStructuralAdaptationSolver2::*)(::std::string const &)) &AbstractStructuralAdaptationSolver2::SetOutputFileName, 
            " " , py::arg("rFilename") )
        .def(
            "SetVesselNetwork", 
            (void(AbstractStructuralAdaptationSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AbstractStructuralAdaptationSolver2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "Solve", 
            (void(AbstractStructuralAdaptationSolver2::*)()) &AbstractStructuralAdaptationSolver2::Solve, 
            " "  )
        .def(
            "Write", 
            (void(AbstractStructuralAdaptationSolver2::*)()) &AbstractStructuralAdaptationSolver2::Write, 
            " "  )
    ;
}
