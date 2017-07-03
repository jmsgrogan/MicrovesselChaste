#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Owen2011MigrationRule.hpp"

#include "Owen2011MigrationRule2.cppwg.hpp"

namespace py = pybind11;
typedef Owen2011MigrationRule<2 > Owen2011MigrationRule2;
;
typedef ::std::vector<int, std::allocator<int> > _std_vectorint_std_allocatorint;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;

class Owen2011MigrationRule2_Overloads : public Owen2011MigrationRule2{
    public:
    using Owen2011MigrationRule2::Owen2011MigrationRule;
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorint_std_allocatorint,
            Owen2011MigrationRule2,
            GetIndices,
            rNodes);
    }

};
void register_Owen2011MigrationRule2_class(py::module &m){
py::class_<Owen2011MigrationRule2 , Owen2011MigrationRule2_Overloads   >(m, "Owen2011MigrationRule2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<Owen2011MigrationRule<2> >(*)()) &Owen2011MigrationRule2::Create, 
            " " )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(Owen2011MigrationRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &Owen2011MigrationRule2::GetIndices, 
            " " , py::arg("rNodes"))
        .def(
            "SetCellChemotacticParameter", 
            (void(Owen2011MigrationRule2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<5, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &Owen2011MigrationRule2::SetCellChemotacticParameter, 
            " " , py::arg("cellChemotacticParameter"))
        .def(
            "SetCellMotilityParameter", 
            (void(Owen2011MigrationRule2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &Owen2011MigrationRule2::SetCellMotilityParameter, 
            " " , py::arg("cellMotility"))
    ;
}
