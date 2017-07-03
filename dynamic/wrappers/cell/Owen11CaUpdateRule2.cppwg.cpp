#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Owen11CaUpdateRule.hpp"

#include "Owen11CaUpdateRule2.cppwg.hpp"

namespace py = pybind11;
typedef Owen11CaUpdateRule<2 > Owen11CaUpdateRule2;
;

class Owen11CaUpdateRule2_Overloads : public Owen11CaUpdateRule2{
    public:
    using Owen11CaUpdateRule2::Owen11CaUpdateRule;
    double EvaluateProbability(unsigned int currentNodeIndex, unsigned int targetNodeIndex, ::CaBasedCellPopulation<2> & rCellPopulation, double dt, double deltaX, ::CellPtr cell) override {
        PYBIND11_OVERLOAD(
            double,
            Owen11CaUpdateRule2,
            EvaluateProbability,
            currentNodeIndex, 
targetNodeIndex, 
rCellPopulation, 
dt, 
deltaX, 
cell);
    }
    void OutputUpdateRuleParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            Owen11CaUpdateRule2,
            OutputUpdateRuleParameters,
            rParamsFile);
    }

};
void register_Owen11CaUpdateRule2_class(py::module &m){
py::class_<Owen11CaUpdateRule2 , Owen11CaUpdateRule2_Overloads   >(m, "Owen11CaUpdateRule2")
        .def(py::init< >())
        .def(
            "EvaluateProbability", 
            (double(Owen11CaUpdateRule2::*)(unsigned int, unsigned int, ::CaBasedCellPopulation<2> &, double, double, ::CellPtr)) &Owen11CaUpdateRule2::EvaluateProbability, 
            " " , py::arg("currentNodeIndex"), py::arg("targetNodeIndex"), py::arg("rCellPopulation"), py::arg("dt"), py::arg("deltaX"), py::arg("cell"))
        .def(
            "GetDiffusionParameter", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(Owen11CaUpdateRule2::*)()) &Owen11CaUpdateRule2::GetDiffusionParameter, 
            " " )
        .def(
            "SetVesselNetwork", 
            (void(Owen11CaUpdateRule2::*)(::std::shared_ptr<VesselNetwork<2> >)) &Owen11CaUpdateRule2::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork"))
        .def(
            "SetGridCalculator", 
            (void(Owen11CaUpdateRule2::*)(::std::shared_ptr<GridCalculator<2> >)) &Owen11CaUpdateRule2::SetGridCalculator, 
            " " , py::arg("pGridCalculator"))
        .def(
            "SetReferenceLengthScale", 
            (void(Owen11CaUpdateRule2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &Owen11CaUpdateRule2::SetReferenceLengthScale, 
            " " , py::arg("referenceLengthScale"))
        .def(
            "SetDiffusionParameter", 
            (void(Owen11CaUpdateRule2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &Owen11CaUpdateRule2::SetDiffusionParameter, 
            " " , py::arg("diffusionParameter"))
        .def(
            "OutputUpdateRuleParameters", 
            (void(Owen11CaUpdateRule2::*)(::out_stream &)) &Owen11CaUpdateRule2::OutputUpdateRuleParameters, 
            " " , py::arg("rParamsFile"))
    ;
}
