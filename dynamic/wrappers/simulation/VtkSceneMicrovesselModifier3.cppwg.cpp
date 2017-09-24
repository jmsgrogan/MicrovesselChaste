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
#include "VtkSceneMicrovesselModifier.hpp"

#include "PythonObjectConverters.hpp"
#include "VtkSceneMicrovesselModifier3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VtkSceneMicrovesselModifier<3 > VtkSceneMicrovesselModifier3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class VtkSceneMicrovesselModifier3_Overloads : public VtkSceneMicrovesselModifier3{
    public:
    using VtkSceneMicrovesselModifier3::VtkSceneMicrovesselModifier;
    void SetupSolve(::std::string outputDirectory) override {
        PYBIND11_OVERLOAD(
            void,
            VtkSceneMicrovesselModifier3,
            SetupSolve,
            outputDirectory);
    }
    void UpdateAtEndOfTimeStep() override {
        PYBIND11_OVERLOAD(
            void,
            VtkSceneMicrovesselModifier3,
            UpdateAtEndOfTimeStep,
            );
    }

};
void register_VtkSceneMicrovesselModifier3_class(py::module &m){
py::class_<VtkSceneMicrovesselModifier3 , VtkSceneMicrovesselModifier3_Overloads , std::shared_ptr<VtkSceneMicrovesselModifier3 >  , AbstractMicrovesselModifier<3>  >(m, "VtkSceneMicrovesselModifier3")
        .def(py::init< >())
        .def(
            "GetVtkScene", 
            (::std::shared_ptr<MicrovesselVtkScene<3> >(VtkSceneMicrovesselModifier3::*)()) &VtkSceneMicrovesselModifier3::GetVtkScene, 
            " "  )
        .def(
            "SetupSolve", 
            (void(VtkSceneMicrovesselModifier3::*)(::std::string)) &VtkSceneMicrovesselModifier3::SetupSolve, 
            " " , py::arg("outputDirectory") )
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(VtkSceneMicrovesselModifier3::*)()) &VtkSceneMicrovesselModifier3::UpdateAtEndOfTimeStep, 
            " "  )
        .def(
            "SetVtkScene", 
            (void(VtkSceneMicrovesselModifier3::*)(::std::shared_ptr<MicrovesselVtkScene<3> >)) &VtkSceneMicrovesselModifier3::SetVtkScene, 
            " " , py::arg("pScene") )
        .def(
            "SetUpdateFrequency", 
            (void(VtkSceneMicrovesselModifier3::*)(unsigned int)) &VtkSceneMicrovesselModifier3::SetUpdateFrequency, 
            " " , py::arg("frequency") )
    ;
}
