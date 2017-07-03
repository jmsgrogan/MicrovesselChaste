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

#include "WallShearStressBasedRegressionSolver2.cppwg.hpp"

namespace py = pybind11;
typedef WallShearStressBasedRegressionSolver<2 > WallShearStressBasedRegressionSolver2;
;

class WallShearStressBasedRegressionSolver2_Overloads : public WallShearStressBasedRegressionSolver2{
    public:
    using WallShearStressBasedRegressionSolver2::WallShearStressBasedRegressionSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            WallShearStressBasedRegressionSolver2,
            Increment,
            );
    }

};
void register_WallShearStressBasedRegressionSolver2_class(py::module &m){
py::class_<WallShearStressBasedRegressionSolver2 , WallShearStressBasedRegressionSolver2_Overloads   >(m, "WallShearStressBasedRegressionSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<WallShearStressBasedRegressionSolver<2> >(*)()) &WallShearStressBasedRegressionSolver2::Create, 
            " " )
        .def(
            "GetLowWallShearStressThreshold", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(WallShearStressBasedRegressionSolver2::*)()) &WallShearStressBasedRegressionSolver2::GetLowWallShearStressThreshold, 
            " " )
        .def(
            "GetMaximumTimeWithLowWallShearStress", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(WallShearStressBasedRegressionSolver2::*)()) &WallShearStressBasedRegressionSolver2::GetMaximumTimeWithLowWallShearStress, 
            " " )
        .def(
            "SetMaximumTimeWithLowWallShearStress", 
            (void(WallShearStressBasedRegressionSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &WallShearStressBasedRegressionSolver2::SetMaximumTimeWithLowWallShearStress, 
            " " , py::arg("time"))
        .def(
            "SetLowWallShearStressThreshold", 
            (void(WallShearStressBasedRegressionSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::mass_base_dimension, boost::units::static_rational<1, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-2, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &WallShearStressBasedRegressionSolver2::SetLowWallShearStressThreshold, 
            " " , py::arg("threshold"))
        .def(
            "Increment", 
            (void(WallShearStressBasedRegressionSolver2::*)()) &WallShearStressBasedRegressionSolver2::Increment, 
            " " )
    ;
}
