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
#include "ShrinkingStimulusCalculator.hpp"

#include "PythonObjectConverters.hpp"
#include "ShrinkingStimulusCalculator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef ShrinkingStimulusCalculator<3 > ShrinkingStimulusCalculator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class ShrinkingStimulusCalculator3_Overloads : public ShrinkingStimulusCalculator3{
    public:
    using ShrinkingStimulusCalculator3::ShrinkingStimulusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ShrinkingStimulusCalculator3,
            Calculate,
            );
    }

};
void register_ShrinkingStimulusCalculator3_class(py::module &m){
py::class_<ShrinkingStimulusCalculator3 , ShrinkingStimulusCalculator3_Overloads , std::shared_ptr<ShrinkingStimulusCalculator3 >  , AbstractVesselNetworkCalculator<3>  >(m, "ShrinkingStimulusCalculator3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ShrinkingStimulusCalculator<3> >(*)()) &ShrinkingStimulusCalculator3::Create, 
            " "  )
        .def(
            "GetStimulus", 
            (::QRate(ShrinkingStimulusCalculator3::*)()) &ShrinkingStimulusCalculator3::GetStimulus, 
            " "  )
        .def(
            "SetStimulus", 
            (void(ShrinkingStimulusCalculator3::*)(::QRate)) &ShrinkingStimulusCalculator3::SetStimulus, 
            " " , py::arg("stimulus") )
        .def(
            "Calculate", 
            (void(ShrinkingStimulusCalculator3::*)()) &ShrinkingStimulusCalculator3::Calculate, 
            " "  )
    ;
}
