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
#include "SegmentFlowProperties.hpp"

#include "SegmentFlowProperties3.cppwg.hpp"

namespace py = pybind11;
typedef SegmentFlowProperties<3 > SegmentFlowProperties3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class SegmentFlowProperties3_Overloads : public SegmentFlowProperties3{
    public:
    using SegmentFlowProperties3::SegmentFlowProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            SegmentFlowProperties3,
            GetOutputData,
            );
    }

};
void register_SegmentFlowProperties3_class(py::module &m){
py::class_<SegmentFlowProperties3 , SegmentFlowProperties3_Overloads , std::shared_ptr<SegmentFlowProperties3 >   >(m, "SegmentFlowProperties3")
        .def(py::init< >())
        .def(
            "GetAntiAngiogenicDrugConcentration", 
            (::QConcentration(SegmentFlowProperties3::*)() const ) &SegmentFlowProperties3::GetAntiAngiogenicDrugConcentration, 
            " "  )
        .def(
            "GetHaematocrit", 
            (::QDimensionless(SegmentFlowProperties3::*)() const ) &SegmentFlowProperties3::GetHaematocrit, 
            " "  )
        .def(
            "GetImpedance", 
            (::QFlowImpedance(SegmentFlowProperties3::*)() const ) &SegmentFlowProperties3::GetImpedance, 
            " "  )
        .def(
            "GetFlowRate", 
            (::QFlowRate(SegmentFlowProperties3::*)() const ) &SegmentFlowProperties3::GetFlowRate, 
            " "  )
        .def(
            "GetViscosity", 
            (::QDynamicViscosity(SegmentFlowProperties3::*)() const ) &SegmentFlowProperties3::GetViscosity, 
            " "  )
        .def(
            "GetWallShearStress", 
            (::QPressure(SegmentFlowProperties3::*)() const ) &SegmentFlowProperties3::GetWallShearStress, 
            " "  )
        .def(
            "GetGrowthStimulus", 
            (::QRate(SegmentFlowProperties3::*)() const ) &SegmentFlowProperties3::GetGrowthStimulus, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(SegmentFlowProperties3::*)() const ) &SegmentFlowProperties3::GetOutputData, 
            " "  )
        .def(
            "SetHaematocrit", 
            (void(SegmentFlowProperties3::*)(::QDimensionless)) &SegmentFlowProperties3::SetHaematocrit, 
            " " , py::arg("haematocrit") )
        .def(
            "SetFlowRate", 
            (void(SegmentFlowProperties3::*)(::QFlowRate)) &SegmentFlowProperties3::SetFlowRate, 
            " " , py::arg("flowRate") )
        .def(
            "SetImpedance", 
            (void(SegmentFlowProperties3::*)(::QFlowImpedance)) &SegmentFlowProperties3::SetImpedance, 
            " " , py::arg("impedance") )
        .def(
            "SetViscosity", 
            (void(SegmentFlowProperties3::*)(::QDynamicViscosity)) &SegmentFlowProperties3::SetViscosity, 
            " " , py::arg("viscosity") )
        .def(
            "SetWallShearStress", 
            (void(SegmentFlowProperties3::*)(::QPressure)) &SegmentFlowProperties3::SetWallShearStress, 
            " " , py::arg("wallShear") )
        .def(
            "SetGrowthStimulus", 
            (void(SegmentFlowProperties3::*)(::QRate)) &SegmentFlowProperties3::SetGrowthStimulus, 
            " " , py::arg("stimulus") )
    ;
}
