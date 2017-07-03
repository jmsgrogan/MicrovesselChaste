#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "ShrinkingStimulusCalculator.hpp"

#include "ShrinkingStimulusCalculator2.cppwg.hpp"

namespace py = pybind11;
typedef ShrinkingStimulusCalculator<2 > ShrinkingStimulusCalculator2;
;

class ShrinkingStimulusCalculator2_Overloads : public ShrinkingStimulusCalculator2{
    public:
    using ShrinkingStimulusCalculator2::ShrinkingStimulusCalculator;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ShrinkingStimulusCalculator2,
            Calculate,
            );
    }

};
void register_ShrinkingStimulusCalculator2_class(py::module &m){
py::class_<ShrinkingStimulusCalculator2 , ShrinkingStimulusCalculator2_Overloads   >(m, "ShrinkingStimulusCalculator2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ShrinkingStimulusCalculator<2> >(*)()) &ShrinkingStimulusCalculator2::Create, 
            " " )
        .def(
            "GetStimulus", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(ShrinkingStimulusCalculator2::*)()) &ShrinkingStimulusCalculator2::GetStimulus, 
            " " )
        .def(
            "SetStimulus", 
            (void(ShrinkingStimulusCalculator2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &ShrinkingStimulusCalculator2::SetStimulus, 
            " " , py::arg("stimulus"))
        .def(
            "Calculate", 
            (void(ShrinkingStimulusCalculator2::*)()) &ShrinkingStimulusCalculator2::Calculate, 
            " " )
    ;
}
