#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "MicrovesselSimulationModifier.hpp"

#include "MicrovesselSimulationModifier3.cppwg.hpp"

namespace py = pybind11;
typedef MicrovesselSimulationModifier<3 > MicrovesselSimulationModifier3;
;

class MicrovesselSimulationModifier3_Overloads : public MicrovesselSimulationModifier3{
    public:
    using MicrovesselSimulationModifier3::MicrovesselSimulationModifier;
    void OutputSimulationModifierParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier3,
            OutputSimulationModifierParameters,
            rParamsFile);
    }
    void SetupSolve(::AbstractCellPopulation<3, 3> & rCellPopulation, ::std::string outputDirectory) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier3,
            SetupSolve,
            rCellPopulation, 
outputDirectory);
    }
    void UpdateAtEndOfTimeStep(::AbstractCellPopulation<3, 3> & rCellPopulation) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier3,
            UpdateAtEndOfTimeStep,
            rCellPopulation);
    }

};
void register_MicrovesselSimulationModifier3_class(py::module &m){
py::class_<MicrovesselSimulationModifier3 , MicrovesselSimulationModifier3_Overloads   >(m, "MicrovesselSimulationModifier3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MicrovesselSimulationModifier<3> >(*)()) &MicrovesselSimulationModifier3::Create, 
            " " )
        .def(
            "OutputSimulationModifierParameters", 
            (void(MicrovesselSimulationModifier3::*)(::out_stream &)) &MicrovesselSimulationModifier3::OutputSimulationModifierParameters, 
            " " , py::arg("rParamsFile"))
        .def(
            "SetCellDataUpdateLabels", 
            (void(MicrovesselSimulationModifier3::*)(::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >)) &MicrovesselSimulationModifier3::SetCellDataUpdateLabels, 
            " " , py::arg("labels"))
        .def(
            "SetMicrovesselSolver", 
            (void(MicrovesselSimulationModifier3::*)(::std::shared_ptr<MicrovesselSolver<3> >)) &MicrovesselSimulationModifier3::SetMicrovesselSolver, 
            " " , py::arg("pSolver"))
        .def(
            "SetGridCalculator", 
            (void(MicrovesselSimulationModifier3::*)(::std::shared_ptr<GridCalculator<3> >)) &MicrovesselSimulationModifier3::SetGridCalculator, 
            " " , py::arg("pGridCalculator"))
        .def(
            "SetupSolve", 
            (void(MicrovesselSimulationModifier3::*)(::AbstractCellPopulation<3, 3> &, ::std::string)) &MicrovesselSimulationModifier3::SetupSolve, 
            " " , py::arg("rCellPopulation"), py::arg("outputDirectory"))
        .def(
            "SetCellPopulationLengthScale", 
            (void(MicrovesselSimulationModifier3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &MicrovesselSimulationModifier3::SetCellPopulationLengthScale, 
            " " , py::arg("cellLengthScale"))
        .def(
            "SetCellPopulationConcentrationScale", 
            (void(MicrovesselSimulationModifier3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &MicrovesselSimulationModifier3::SetCellPopulationConcentrationScale, 
            " " , py::arg("cellConcentrationScale"))
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(MicrovesselSimulationModifier3::*)(::AbstractCellPopulation<3, 3> &)) &MicrovesselSimulationModifier3::UpdateAtEndOfTimeStep, 
            " " , py::arg("rCellPopulation"))
        .def(
            "UpdateCellData", 
            (void(MicrovesselSimulationModifier3::*)(::AbstractCellPopulation<3, 3> &)) &MicrovesselSimulationModifier3::UpdateCellData, 
            " " , py::arg("rCellPopulation"))
    ;
}
