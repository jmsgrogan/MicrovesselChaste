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

#include "VesselFlowProperties3.cppwg.hpp"

namespace py = pybind11;
typedef VesselFlowProperties<3 > VesselFlowProperties3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class VesselFlowProperties3_Overloads : public VesselFlowProperties3{
    public:
    using VesselFlowProperties3::VesselFlowProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            VesselFlowProperties3,
            GetOutputData,
            );
    }

};
void register_VesselFlowProperties3_class(py::module &m){
py::class_<VesselFlowProperties3 , VesselFlowProperties3_Overloads , std::shared_ptr<VesselFlowProperties3 >   >(m, "VesselFlowProperties3")
        .def(py::init< >())
        .def(
            "CheckSegments", 
            (void(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::CheckSegments, 
            " "  )
        .def(
            "GetAntiAngiogenicDrugConcentration", 
            (::QConcentration(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetAntiAngiogenicDrugConcentration, 
            " "  )
        .def(
            "GetHaematocrit", 
            (::QDimensionless(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetHaematocrit, 
            " "  )
        .def(
            "GetImpedance", 
            (::QFlowImpedance(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetImpedance, 
            " "  )
        .def(
            "GetFlowRate", 
            (::QFlowRate(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetFlowRate, 
            " "  )
        .def(
            "GetViscosity", 
            (::QDynamicViscosity(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetViscosity, 
            " "  )
        .def(
            "GetWallShearStress", 
            (::QPressure(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetWallShearStress, 
            " "  )
        .def(
            "GetGrowthStimulus", 
            (::QRate(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetGrowthStimulus, 
            " "  )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetOutputData, 
            " "  )
        .def(
            "GetRegressionTime", 
            (::QTime(VesselFlowProperties3::*)() const ) &VesselFlowProperties3::GetRegressionTime, 
            " "  )
        .def(
            "HasRegressionTimerStarted", 
            (bool(VesselFlowProperties3::*)()) &VesselFlowProperties3::HasRegressionTimerStarted, 
            " "  )
        .def(
            "HasVesselRegressed", 
            (bool(VesselFlowProperties3::*)(::QTime)) &VesselFlowProperties3::HasVesselRegressed, 
            " " , py::arg("simulationReferenceTime") )
        .def(
            "ResetRegressionTimer", 
            (void(VesselFlowProperties3::*)()) &VesselFlowProperties3::ResetRegressionTimer, 
            " "  )
        .def(
            "SetHaematocrit", 
            (void(VesselFlowProperties3::*)(::QDimensionless)) &VesselFlowProperties3::SetHaematocrit, 
            " " , py::arg("haematocrit") )
        .def(
            "SetFlowRate", 
            (void(VesselFlowProperties3::*)(::QFlowRate)) &VesselFlowProperties3::SetFlowRate, 
            " " , py::arg("flowRate") )
        .def(
            "SetImpedance", 
            (void(VesselFlowProperties3::*)(::QFlowImpedance)) &VesselFlowProperties3::SetImpedance, 
            " " , py::arg("impedance") )
        .def(
            "SetViscosity", 
            (void(VesselFlowProperties3::*)(::QDynamicViscosity)) &VesselFlowProperties3::SetViscosity, 
            " " , py::arg("viscosity") )
        .def(
            "SetWallShearStress", 
            (void(VesselFlowProperties3::*)(::QPressure)) &VesselFlowProperties3::SetWallShearStress, 
            " " , py::arg("wallShear") )
        .def(
            "SetGrowthStimulus", 
            (void(VesselFlowProperties3::*)(::QRate)) &VesselFlowProperties3::SetGrowthStimulus, 
            " " , py::arg("stimulus") )
        .def(
            "SetTimeUntilRegression", 
            (void(VesselFlowProperties3::*)(::QTime, ::QTime)) &VesselFlowProperties3::SetTimeUntilRegression, 
            " " , py::arg("time"), py::arg("simulationReferenceTime") )
        .def(
            "SetRegressionTime", 
            (void(VesselFlowProperties3::*)(::QTime)) &VesselFlowProperties3::SetRegressionTime, 
            " " , py::arg("time") )
        .def(
            "UpdateSegments", 
            (void(VesselFlowProperties3::*)(::std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >)) &VesselFlowProperties3::UpdateSegments, 
            " " , py::arg("segments") )
    ;
}
