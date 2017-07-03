#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AlarconHaematocritSolver.hpp"

#include "AlarconHaematocritSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AlarconHaematocritSolver<2 > AlarconHaematocritSolver2;
;

class AlarconHaematocritSolver2_Overloads : public AlarconHaematocritSolver2{
    public:
    using AlarconHaematocritSolver2::AlarconHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            AlarconHaematocritSolver2,
            Calculate,
            );
    }

};
void register_AlarconHaematocritSolver2_class(py::module &m){
py::class_<AlarconHaematocritSolver2 , AlarconHaematocritSolver2_Overloads   >(m, "AlarconHaematocritSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AlarconHaematocritSolver<2> >(*)()) &AlarconHaematocritSolver2::Create, 
            " " )
        .def(
            "Calculate", 
            (void(AlarconHaematocritSolver2::*)()) &AlarconHaematocritSolver2::Calculate, 
            " " )
        .def(
            "SetTHR", 
            (void(AlarconHaematocritSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AlarconHaematocritSolver2::SetTHR, 
            " " , py::arg("thr"))
        .def(
            "SetAlpha", 
            (void(AlarconHaematocritSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AlarconHaematocritSolver2::SetAlpha, 
            " " , py::arg("alpha"))
        .def(
            "SetHaematocrit", 
            (void(AlarconHaematocritSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AlarconHaematocritSolver2::SetHaematocrit, 
            " " , py::arg("haematocrit"))
    ;
}
