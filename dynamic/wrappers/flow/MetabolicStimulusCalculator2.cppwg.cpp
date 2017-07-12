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
#include "MetabolicStimulusCalculator.hpp"

#include "MetabolicStimulusCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef MetabolicStimulusCalculator<2 > MetabolicStimulusCalculator2;
;

class MetabolicStimulusCalculator2_Overloads : public MetabolicStimulusCalculator2{
    public:
    using MetabolicStimulusCalculator2::MetabolicStimulusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            MetabolicStimulusCalculator2,
            Calculate,
            );
    }

};
void register_MetabolicStimulusCalculator2_class(py::module &m){
py::class_<MetabolicStimulusCalculator2 , MetabolicStimulusCalculator2_Overloads   >(m, "MetabolicStimulusCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MetabolicStimulusCalculator<2> >(*)()) &MetabolicStimulusCalculator2::Create, 
            " "  )
        .def(
            "GetQRef", 
            (::QFlowRate(MetabolicStimulusCalculator2::*)()) &MetabolicStimulusCalculator2::GetQRef, 
            " "  )
        .def(
            "GetKm", 
            (::QRate(MetabolicStimulusCalculator2::*)()) &MetabolicStimulusCalculator2::GetKm, 
            " "  )
        .def(
            "GetMaxStimulus", 
            (::QRate(MetabolicStimulusCalculator2::*)()) &MetabolicStimulusCalculator2::GetMaxStimulus, 
            " "  )
        .def(
            "SetQRef", 
            (void(MetabolicStimulusCalculator2::*)(::QFlowRate)) &MetabolicStimulusCalculator2::SetQRef, 
            " " , py::arg("qRef") )
        .def(
            "SetKm", 
            (void(MetabolicStimulusCalculator2::*)(::QRate)) &MetabolicStimulusCalculator2::SetKm, 
            " " , py::arg("km") )
        .def(
            "SetMaxStimulus", 
            (void(MetabolicStimulusCalculator2::*)(::QRate)) &MetabolicStimulusCalculator2::SetMaxStimulus, 
            " " , py::arg("maxStimulus") )
        .def(
            "Calculate", 
            (void(MetabolicStimulusCalculator2::*)()) &MetabolicStimulusCalculator2::Calculate, 
            " "  )
    ;
}
