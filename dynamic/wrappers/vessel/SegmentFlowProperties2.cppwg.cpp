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

#include "SegmentFlowProperties2.cppwg.hpp"

namespace py = pybind11;
typedef SegmentFlowProperties<2 > SegmentFlowProperties2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class SegmentFlowProperties2_Overloads : public SegmentFlowProperties2{
    public:
    using SegmentFlowProperties2::SegmentFlowProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            SegmentFlowProperties2,
            GetOutputData,
            );
    }

};
void register_SegmentFlowProperties2_class(py::module &m){
py::class_<SegmentFlowProperties2 , SegmentFlowProperties2_Overloads , std::shared_ptr<SegmentFlowProperties2 >   >(m, "SegmentFlowProperties2")
        .def(py::init< >())
        .def(
            "GetAntiAngiogenicDrugConcentration", 
            (::QConcentration(SegmentFlowProperties2::*)() const ) &SegmentFlowProperties2::GetAntiAngiogenicDrugConcentration, 
            " "  )
        .def(
            "GetHaematocrit", 
            (::QDimensionless(SegmentFlowProperties2::*)() const ) &SegmentFlowProperties2::GetHaematocrit, 
            " "  )
        .def(
            "GetImpedance", 
            (::QFlowImpedance(SegmentFlowProperties2::*)() const ) &SegmentFlowProperties2::GetImpedance, 
            " "  )
        .def(
            "GetFlowRate", 
            (::QFlowRate(SegmentFlowProperties2::*)() const ) &SegmentFlowProperties2::GetFlowRate, 
            " "  )
        .def(
            "GetViscosity", 
            (::QDynamicViscosity(SegmentFlowProperties2::*)() const ) &SegmentFlowProperties2::GetViscosity, 
            " "  )
        .def(
            "GetWallShearStress", 
            (::QPressure(SegmentFlowProperties2::*)() const ) &SegmentFlowProperties2::GetWallShearStress, 
            " "  )
        .def(
            "GetGrowthStimulus", 
            (::QRate(SegmentFlowProperties2::*)() const ) &SegmentFlowProperties2::GetGrowthStimulus, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(SegmentFlowProperties2::*)() const ) &SegmentFlowProperties2::GetOutputData, 
            " "  )
        .def(
            "SetHaematocrit", 
            (void(SegmentFlowProperties2::*)(::QDimensionless)) &SegmentFlowProperties2::SetHaematocrit, 
            " " , py::arg("haematocrit") )
        .def(
            "SetFlowRate", 
            (void(SegmentFlowProperties2::*)(::QFlowRate)) &SegmentFlowProperties2::SetFlowRate, 
            " " , py::arg("flowRate") )
        .def(
            "SetImpedance", 
            (void(SegmentFlowProperties2::*)(::QFlowImpedance)) &SegmentFlowProperties2::SetImpedance, 
            " " , py::arg("impedance") )
        .def(
            "SetViscosity", 
            (void(SegmentFlowProperties2::*)(::QDynamicViscosity)) &SegmentFlowProperties2::SetViscosity, 
            " " , py::arg("viscosity") )
        .def(
            "SetWallShearStress", 
            (void(SegmentFlowProperties2::*)(::QPressure)) &SegmentFlowProperties2::SetWallShearStress, 
            " " , py::arg("wallShear") )
        .def(
            "SetGrowthStimulus", 
            (void(SegmentFlowProperties2::*)(::QRate)) &SegmentFlowProperties2::SetGrowthStimulus, 
            " " , py::arg("stimulus") )
    ;
}
