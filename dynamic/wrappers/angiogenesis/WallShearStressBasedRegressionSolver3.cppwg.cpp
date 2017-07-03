#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "WallShearStressBasedRegressionSolver.hpp"

#include "WallShearStressBasedRegressionSolver3.cppwg.hpp"

namespace py = pybind11;
typedef WallShearStressBasedRegressionSolver<3 > WallShearStressBasedRegressionSolver3;
;

class WallShearStressBasedRegressionSolver3_Overloads : public WallShearStressBasedRegressionSolver3{
    public:
    using WallShearStressBasedRegressionSolver3::WallShearStressBasedRegressionSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            WallShearStressBasedRegressionSolver3,
            Increment,
            );
    }

};
void register_WallShearStressBasedRegressionSolver3_class(py::module &m){
py::class_<WallShearStressBasedRegressionSolver3 , WallShearStressBasedRegressionSolver3_Overloads   >(m, "WallShearStressBasedRegressionSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<WallShearStressBasedRegressionSolver<3> >(*)()) &WallShearStressBasedRegressionSolver3::Create, 
            " " )
        .def(
            "GetLowWallShearStressThreshold", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(WallShearStressBasedRegressionSolver3::*)()) &WallShearStressBasedRegressionSolver3::GetLowWallShearStressThreshold, 
            " " )
        .def(
            "GetMaximumTimeWithLowWallShearStress", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(WallShearStressBasedRegressionSolver3::*)()) &WallShearStressBasedRegressionSolver3::GetMaximumTimeWithLowWallShearStress, 
            " " )
        .def(
            "SetMaximumTimeWithLowWallShearStress", 
            (void(WallShearStressBasedRegressionSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &WallShearStressBasedRegressionSolver3::SetMaximumTimeWithLowWallShearStress, 
            " " , py::arg("time"))
        .def(
            "SetLowWallShearStressThreshold", 
            (void(WallShearStressBasedRegressionSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &WallShearStressBasedRegressionSolver3::SetLowWallShearStressThreshold, 
            " " , py::arg("threshold"))
        .def(
            "Increment", 
            (void(WallShearStressBasedRegressionSolver3::*)()) &WallShearStressBasedRegressionSolver3::Increment, 
            " " )
    ;
}
