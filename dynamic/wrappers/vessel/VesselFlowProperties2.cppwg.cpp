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
#include "VesselFlowProperties.hpp"

#include "VesselFlowProperties2.cppwg.hpp"

namespace py = pybind11;
typedef VesselFlowProperties<2 > VesselFlowProperties2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class VesselFlowProperties2_Overloads : public VesselFlowProperties2{
    public:
    using VesselFlowProperties2::VesselFlowProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            VesselFlowProperties2,
            GetOutputData,
            );
    }

};
void register_VesselFlowProperties2_class(py::module &m){
py::class_<VesselFlowProperties2 , VesselFlowProperties2_Overloads , std::shared_ptr<VesselFlowProperties2 >   >(m, "VesselFlowProperties2")
        .def(py::init< >())
        .def(
            "CheckSegments", 
            (void(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::CheckSegments, 
            " "  )
        .def(
            "GetAntiAngiogenicDrugConcentration", 
            (::QConcentration(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetAntiAngiogenicDrugConcentration, 
            " "  )
        .def(
            "GetHaematocrit", 
            (::QDimensionless(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetHaematocrit, 
            " "  )
        .def(
            "GetImpedance", 
            (::QFlowImpedance(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetImpedance, 
            " "  )
        .def(
            "GetFlowRate", 
            (::QFlowRate(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetFlowRate, 
            " "  )
        .def(
            "GetViscosity", 
            (::QDynamicViscosity(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetViscosity, 
            " "  )
        .def(
            "GetWallShearStress", 
            (::QPressure(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetWallShearStress, 
            " "  )
        .def(
            "GetGrowthStimulus", 
            (::QRate(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetGrowthStimulus, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetOutputData, 
            " "  )
        .def(
            "GetRegressionTime", 
            (::QTime(VesselFlowProperties2::*)() const ) &VesselFlowProperties2::GetRegressionTime, 
            " "  )
        .def(
            "HasRegressionTimerStarted", 
            (bool(VesselFlowProperties2::*)()) &VesselFlowProperties2::HasRegressionTimerStarted, 
            " "  )
        .def(
            "HasVesselRegressed", 
            (bool(VesselFlowProperties2::*)(::QTime)) &VesselFlowProperties2::HasVesselRegressed, 
            " " , py::arg("simulationReferenceTime") )
        .def(
            "ResetRegressionTimer", 
            (void(VesselFlowProperties2::*)()) &VesselFlowProperties2::ResetRegressionTimer, 
            " "  )
        .def(
            "SetHaematocrit", 
            (void(VesselFlowProperties2::*)(::QDimensionless)) &VesselFlowProperties2::SetHaematocrit, 
            " " , py::arg("haematocrit") )
        .def(
            "SetFlowRate", 
            (void(VesselFlowProperties2::*)(::QFlowRate)) &VesselFlowProperties2::SetFlowRate, 
            " " , py::arg("flowRate") )
        .def(
            "SetImpedance", 
            (void(VesselFlowProperties2::*)(::QFlowImpedance)) &VesselFlowProperties2::SetImpedance, 
            " " , py::arg("impedance") )
        .def(
            "SetViscosity", 
            (void(VesselFlowProperties2::*)(::QDynamicViscosity)) &VesselFlowProperties2::SetViscosity, 
            " " , py::arg("viscosity") )
        .def(
            "SetWallShearStress", 
            (void(VesselFlowProperties2::*)(::QPressure)) &VesselFlowProperties2::SetWallShearStress, 
            " " , py::arg("wallShear") )
        .def(
            "SetGrowthStimulus", 
            (void(VesselFlowProperties2::*)(::QRate)) &VesselFlowProperties2::SetGrowthStimulus, 
            " " , py::arg("stimulus") )
        .def(
            "SetTimeUntilRegression", 
            (void(VesselFlowProperties2::*)(::QTime, ::QTime)) &VesselFlowProperties2::SetTimeUntilRegression, 
            " " , py::arg("time"), py::arg("simulationReferenceTime") )
        .def(
            "SetRegressionTime", 
            (void(VesselFlowProperties2::*)(::QTime)) &VesselFlowProperties2::SetRegressionTime, 
            " " , py::arg("time") )
        .def(
            "UpdateSegments", 
            (void(VesselFlowProperties2::*)(::std::vector<std::shared_ptr<VesselSegment<2> >, std::allocator<std::shared_ptr<VesselSegment<2> > > >)) &VesselFlowProperties2::UpdateSegments, 
            " " , py::arg("segments") )
    ;
}
