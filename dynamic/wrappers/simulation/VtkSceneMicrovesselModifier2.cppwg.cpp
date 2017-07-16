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

#include "VtkSceneMicrovesselModifier2.cppwg.hpp"

namespace py = pybind11;
typedef VtkSceneMicrovesselModifier<2 > VtkSceneMicrovesselModifier2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class VtkSceneMicrovesselModifier2_Overloads : public VtkSceneMicrovesselModifier2{
    public:
    using VtkSceneMicrovesselModifier2::VtkSceneMicrovesselModifier;
    void SetupSolve(::std::string outputDirectory) override {
        PYBIND11_OVERLOAD(
            void,
            VtkSceneMicrovesselModifier2,
            SetupSolve,
            outputDirectory);
    }
    void UpdateAtEndOfTimeStep() override {
        PYBIND11_OVERLOAD(
            void,
            VtkSceneMicrovesselModifier2,
            UpdateAtEndOfTimeStep,
            );
    }

};
void register_VtkSceneMicrovesselModifier2_class(py::module &m){
py::class_<VtkSceneMicrovesselModifier2 , VtkSceneMicrovesselModifier2_Overloads , std::shared_ptr<VtkSceneMicrovesselModifier2 >  , AbstractMicrovesselModifier<2>  >(m, "VtkSceneMicrovesselModifier2")
        .def(py::init< >())
        .def(
            "GetVtkScene", 
            (::std::shared_ptr<MicrovesselVtkScene<2> >(VtkSceneMicrovesselModifier2::*)()) &VtkSceneMicrovesselModifier2::GetVtkScene, 
            " "  )
        .def(
            "SetupSolve", 
            (void(VtkSceneMicrovesselModifier2::*)(::std::string)) &VtkSceneMicrovesselModifier2::SetupSolve, 
            " " , py::arg("outputDirectory") )
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(VtkSceneMicrovesselModifier2::*)()) &VtkSceneMicrovesselModifier2::UpdateAtEndOfTimeStep, 
            " "  )
        .def(
            "SetVtkScene", 
            (void(VtkSceneMicrovesselModifier2::*)(::std::shared_ptr<MicrovesselVtkScene<2> >)) &VtkSceneMicrovesselModifier2::SetVtkScene, 
            " " , py::arg("pScene") )
        .def(
            "SetUpdateFrequency", 
            (void(VtkSceneMicrovesselModifier2::*)(unsigned int)) &VtkSceneMicrovesselModifier2::SetUpdateFrequency, 
            " " , py::arg("frequency") )
    ;
}
