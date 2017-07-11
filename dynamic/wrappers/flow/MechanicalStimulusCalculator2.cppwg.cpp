#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "MechanicalStimulusCalculator.hpp"

#include "MechanicalStimulusCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef MechanicalStimulusCalculator<2 > MechanicalStimulusCalculator2;
;

class MechanicalStimulusCalculator2_Overloads : public MechanicalStimulusCalculator2{
    public:
    using MechanicalStimulusCalculator2::MechanicalStimulusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            MechanicalStimulusCalculator2,
            Calculate,
            );
    }

};
void register_MechanicalStimulusCalculator2_class(py::module &m){
py::class_<MechanicalStimulusCalculator2 , MechanicalStimulusCalculator2_Overloads   >(m, "MechanicalStimulusCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MechanicalStimulusCalculator<2> >(*)()) &MechanicalStimulusCalculator2::Create, 
            " "  )
        .def(
            "GetTauP", 
            (::QPressure(MechanicalStimulusCalculator2::*)()) &MechanicalStimulusCalculator2::GetTauP, 
            " "  )
        .def(
            "GetTauReference", 
            (::QPressure(MechanicalStimulusCalculator2::*)()) &MechanicalStimulusCalculator2::GetTauReference, 
            " "  )
        .def(
            "SetTauRef", 
            (void(MechanicalStimulusCalculator2::*)(::QPressure)) &MechanicalStimulusCalculator2::SetTauRef, 
            " " , py::arg("tauRef") )
        .def(
            "SetTauP", 
            (void(MechanicalStimulusCalculator2::*)(::QPressure)) &MechanicalStimulusCalculator2::SetTauP, 
            " " , py::arg("tauP") )
        .def(
            "Calculate", 
            (void(MechanicalStimulusCalculator2::*)()) &MechanicalStimulusCalculator2::Calculate, 
            " "  )
    ;
}