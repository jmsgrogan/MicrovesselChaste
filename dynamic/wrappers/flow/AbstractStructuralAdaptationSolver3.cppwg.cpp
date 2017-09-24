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

#include "PythonObjectConverters.hpp"
#include "AbstractStructuralAdaptationSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractStructuralAdaptationSolver<3 > AbstractStructuralAdaptationSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractStructuralAdaptationSolver3_Overloads : public AbstractStructuralAdaptationSolver3{
    public:
    using AbstractStructuralAdaptationSolver3::AbstractStructuralAdaptationSolver;
    void Iterate() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractStructuralAdaptationSolver3,
            Iterate,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractStructuralAdaptationSolver3,
            Write,
            );
    }

};
void register_AbstractStructuralAdaptationSolver3_class(py::module &m){
py::class_<AbstractStructuralAdaptationSolver3 , AbstractStructuralAdaptationSolver3_Overloads , std::shared_ptr<AbstractStructuralAdaptationSolver3 >   >(m, "AbstractStructuralAdaptationSolver3")
        .def(py::init< >())
        .def(
            "GetTolerance", 
            (double(AbstractStructuralAdaptationSolver3::*)() const ) &AbstractStructuralAdaptationSolver3::GetTolerance, 
            " "  )
        .def(
            "GetWriteOutput", 
            (bool(AbstractStructuralAdaptationSolver3::*)() const ) &AbstractStructuralAdaptationSolver3::GetWriteOutput, 
            " "  )
        .def(
            "GetOutputFileName", 
            (::std::string(AbstractStructuralAdaptationSolver3::*)() const ) &AbstractStructuralAdaptationSolver3::GetOutputFileName, 
            " "  )
        .def(
            "GetTimeIncrement", 
            (::QTime(AbstractStructuralAdaptationSolver3::*)() const ) &AbstractStructuralAdaptationSolver3::GetTimeIncrement, 
            " "  )
        .def(
            "Iterate", 
            (void(AbstractStructuralAdaptationSolver3::*)()) &AbstractStructuralAdaptationSolver3::Iterate, 
            " "  )
        .def(
            "SetTolerance", 
            (void(AbstractStructuralAdaptationSolver3::*)(double)) &AbstractStructuralAdaptationSolver3::SetTolerance, 
            " " , py::arg("tolerance") )
        .def(
            "SetTimeIncrement", 
            (void(AbstractStructuralAdaptationSolver3::*)(::QTime)) &AbstractStructuralAdaptationSolver3::SetTimeIncrement, 
            " " , py::arg("timeIncrement") )
        .def(
            "SetMaxIterations", 
            (void(AbstractStructuralAdaptationSolver3::*)(unsigned int)) &AbstractStructuralAdaptationSolver3::SetMaxIterations, 
            " " , py::arg("iterations") )
        .def(
            "SetWriteOutput", 
            (void(AbstractStructuralAdaptationSolver3::*)(bool)) &AbstractStructuralAdaptationSolver3::SetWriteOutput, 
            " " , py::arg("writeFlag") )
        .def(
            "SetOutputFileName", 
            (void(AbstractStructuralAdaptationSolver3::*)(::std::string const &)) &AbstractStructuralAdaptationSolver3::SetOutputFileName, 
            " " , py::arg("rFilename") )
        .def(
            "SetVesselNetwork", 
            (void(AbstractStructuralAdaptationSolver3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AbstractStructuralAdaptationSolver3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "Solve", 
            (void(AbstractStructuralAdaptationSolver3::*)()) &AbstractStructuralAdaptationSolver3::Solve, 
            " "  )
        .def(
            "Write", 
            (void(AbstractStructuralAdaptationSolver3::*)()) &AbstractStructuralAdaptationSolver3::Write, 
            " "  )
    ;
}
