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

#include "PythonObjectConverters.hpp"
#include "MetabolicStimulusCalculator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef MetabolicStimulusCalculator<3 > MetabolicStimulusCalculator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class MetabolicStimulusCalculator3_Overloads : public MetabolicStimulusCalculator3{
    public:
    using MetabolicStimulusCalculator3::MetabolicStimulusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            MetabolicStimulusCalculator3,
            Calculate,
            );
    }

};
void register_MetabolicStimulusCalculator3_class(py::module &m){
py::class_<MetabolicStimulusCalculator3 , MetabolicStimulusCalculator3_Overloads , std::shared_ptr<MetabolicStimulusCalculator3 >  , AbstractVesselNetworkCalculator<3>  >(m, "MetabolicStimulusCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MetabolicStimulusCalculator<3> >(*)()) &MetabolicStimulusCalculator3::Create, 
            " "  )
        .def(
            "GetQRef", 
            (::QFlowRate(MetabolicStimulusCalculator3::*)()) &MetabolicStimulusCalculator3::GetQRef, 
            " "  )
        .def(
            "GetKm", 
            (::QRate(MetabolicStimulusCalculator3::*)()) &MetabolicStimulusCalculator3::GetKm, 
            " "  )
        .def(
            "GetMaxStimulus", 
            (::QRate(MetabolicStimulusCalculator3::*)()) &MetabolicStimulusCalculator3::GetMaxStimulus, 
            " "  )
        .def(
            "SetQRef", 
            (void(MetabolicStimulusCalculator3::*)(::QFlowRate)) &MetabolicStimulusCalculator3::SetQRef, 
            " " , py::arg("qRef") )
        .def(
            "SetKm", 
            (void(MetabolicStimulusCalculator3::*)(::QRate)) &MetabolicStimulusCalculator3::SetKm, 
            " " , py::arg("km") )
        .def(
            "SetMaxStimulus", 
            (void(MetabolicStimulusCalculator3::*)(::QRate)) &MetabolicStimulusCalculator3::SetMaxStimulus, 
            " " , py::arg("maxStimulus") )
        .def(
            "Calculate", 
            (void(MetabolicStimulusCalculator3::*)()) &MetabolicStimulusCalculator3::Calculate, 
            " "  )
    ;
}
