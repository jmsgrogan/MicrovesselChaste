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
#include "MechanicalStimulusCalculator.hpp"

#include "MechanicalStimulusCalculator3.cppwg.hpp"

namespace py = pybind11;
typedef MechanicalStimulusCalculator<3 > MechanicalStimulusCalculator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class MechanicalStimulusCalculator3_Overloads : public MechanicalStimulusCalculator3{
    public:
    using MechanicalStimulusCalculator3::MechanicalStimulusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            MechanicalStimulusCalculator3,
            Calculate,
            );
    }

};
void register_MechanicalStimulusCalculator3_class(py::module &m){
py::class_<MechanicalStimulusCalculator3 , MechanicalStimulusCalculator3_Overloads , std::shared_ptr<MechanicalStimulusCalculator3 >  , AbstractVesselNetworkCalculator<3>  >(m, "MechanicalStimulusCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MechanicalStimulusCalculator<3> >(*)()) &MechanicalStimulusCalculator3::Create, 
            " "  )
        .def(
            "GetTauP", 
            (::QPressure(MechanicalStimulusCalculator3::*)()) &MechanicalStimulusCalculator3::GetTauP, 
            " "  )
        .def(
            "GetTauReference", 
            (::QPressure(MechanicalStimulusCalculator3::*)()) &MechanicalStimulusCalculator3::GetTauReference, 
            " "  )
        .def(
            "SetTauRef", 
            (void(MechanicalStimulusCalculator3::*)(::QPressure)) &MechanicalStimulusCalculator3::SetTauRef, 
            " " , py::arg("tauRef") )
        .def(
            "SetTauP", 
            (void(MechanicalStimulusCalculator3::*)(::QPressure)) &MechanicalStimulusCalculator3::SetTauP, 
            " " , py::arg("tauP") )
        .def(
            "Calculate", 
            (void(MechanicalStimulusCalculator3::*)()) &MechanicalStimulusCalculator3::Calculate, 
            " "  )
    ;
}
