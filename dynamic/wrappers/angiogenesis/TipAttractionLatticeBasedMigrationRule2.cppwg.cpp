#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "TipAttractionLatticeBasedMigrationRule.hpp"

#include "TipAttractionLatticeBasedMigrationRule2.cppwg.hpp"

namespace py = pybind11;
typedef TipAttractionLatticeBasedMigrationRule<2 > TipAttractionLatticeBasedMigrationRule2;
;
typedef ::std::vector<int, std::allocator<int> > _std_vectorint_std_allocatorint;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;

class TipAttractionLatticeBasedMigrationRule2_Overloads : public TipAttractionLatticeBasedMigrationRule2{
    public:
    using TipAttractionLatticeBasedMigrationRule2::TipAttractionLatticeBasedMigrationRule;
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorint_std_allocatorint,
            TipAttractionLatticeBasedMigrationRule2,
            GetIndices,
            rNodes);
    }

};
void register_TipAttractionLatticeBasedMigrationRule2_class(py::module &m){
py::class_<TipAttractionLatticeBasedMigrationRule2 , TipAttractionLatticeBasedMigrationRule2_Overloads   >(m, "TipAttractionLatticeBasedMigrationRule2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<TipAttractionLatticeBasedMigrationRule<2> >(*)()) &TipAttractionLatticeBasedMigrationRule2::Create, 
            " " )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(TipAttractionLatticeBasedMigrationRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &TipAttractionLatticeBasedMigrationRule2::GetIndices, 
            " " , py::arg("rNodes"))
        .def(
            "SetCellChemotacticParameter", 
            (void(TipAttractionLatticeBasedMigrationRule2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<5, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> > >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &TipAttractionLatticeBasedMigrationRule2::SetCellChemotacticParameter, 
            " " , py::arg("cellChemotacticParameter"))
        .def(
            "SetCellMotilityParameter", 
            (void(TipAttractionLatticeBasedMigrationRule2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &TipAttractionLatticeBasedMigrationRule2::SetCellMotilityParameter, 
            " " , py::arg("cellMotility"))
        .def(
            "SetUseTipAttraction", 
            (void(TipAttractionLatticeBasedMigrationRule2::*)(bool)) &TipAttractionLatticeBasedMigrationRule2::SetUseTipAttraction, 
            " " , py::arg("useTipAttraction"))
    ;
}
