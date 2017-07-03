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

#include "MicrovesselSimulationModifier2.cppwg.hpp"

namespace py = pybind11;
typedef MicrovesselSimulationModifier<2 > MicrovesselSimulationModifier2;
;

class MicrovesselSimulationModifier2_Overloads : public MicrovesselSimulationModifier2{
    public:
    using MicrovesselSimulationModifier2::MicrovesselSimulationModifier;
    void OutputSimulationModifierParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier2,
            OutputSimulationModifierParameters,
            rParamsFile);
    }
    void SetupSolve(::AbstractCellPopulation<2, 2> & rCellPopulation, ::std::string outputDirectory) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier2,
            SetupSolve,
            rCellPopulation, 
outputDirectory);
    }
    void UpdateAtEndOfTimeStep(::AbstractCellPopulation<2, 2> & rCellPopulation) override {
        PYBIND11_OVERLOAD(
            void,
            MicrovesselSimulationModifier2,
            UpdateAtEndOfTimeStep,
            rCellPopulation);
    }

};
void register_MicrovesselSimulationModifier2_class(py::module &m){
py::class_<MicrovesselSimulationModifier2 , MicrovesselSimulationModifier2_Overloads   >(m, "MicrovesselSimulationModifier2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<MicrovesselSimulationModifier<2> >(*)()) &MicrovesselSimulationModifier2::Create, 
            " " )
        .def(
            "OutputSimulationModifierParameters", 
            (void(MicrovesselSimulationModifier2::*)(::out_stream &)) &MicrovesselSimulationModifier2::OutputSimulationModifierParameters, 
            " " , py::arg("rParamsFile"))
        .def(
            "SetCellDataUpdateLabels", 
            (void(MicrovesselSimulationModifier2::*)(::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >)) &MicrovesselSimulationModifier2::SetCellDataUpdateLabels, 
            " " , py::arg("labels"))
        .def(
            "SetMicrovesselSolver", 
            (void(MicrovesselSimulationModifier2::*)(::std::shared_ptr<MicrovesselSolver<2> >)) &MicrovesselSimulationModifier2::SetMicrovesselSolver, 
            " " , py::arg("pSolver"))
        .def(
            "SetGridCalculator", 
            (void(MicrovesselSimulationModifier2::*)(::std::shared_ptr<GridCalculator<2> >)) &MicrovesselSimulationModifier2::SetGridCalculator, 
            " " , py::arg("pGridCalculator"))
        .def(
            "SetupSolve", 
            (void(MicrovesselSimulationModifier2::*)(::AbstractCellPopulation<2, 2> &, ::std::string)) &MicrovesselSimulationModifier2::SetupSolve, 
            " " , py::arg("rCellPopulation"), py::arg("outputDirectory"))
        .def(
            "SetCellPopulationLengthScale", 
            (void(MicrovesselSimulationModifier2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &MicrovesselSimulationModifier2::SetCellPopulationLengthScale, 
            " " , py::arg("cellLengthScale"))
        .def(
            "SetCellPopulationConcentrationScale", 
            (void(MicrovesselSimulationModifier2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &MicrovesselSimulationModifier2::SetCellPopulationConcentrationScale, 
            " " , py::arg("cellConcentrationScale"))
        .def(
            "UpdateAtEndOfTimeStep", 
            (void(MicrovesselSimulationModifier2::*)(::AbstractCellPopulation<2, 2> &)) &MicrovesselSimulationModifier2::UpdateAtEndOfTimeStep, 
            " " , py::arg("rCellPopulation"))
        .def(
            "UpdateCellData", 
            (void(MicrovesselSimulationModifier2::*)(::AbstractCellPopulation<2, 2> &)) &MicrovesselSimulationModifier2::UpdateCellData, 
            " " , py::arg("rCellPopulation"))
    ;
}
