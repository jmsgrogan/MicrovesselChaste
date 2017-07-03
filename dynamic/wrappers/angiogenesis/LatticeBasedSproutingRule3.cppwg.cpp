#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "LatticeBasedSproutingRule.hpp"

#include "LatticeBasedSproutingRule3.cppwg.hpp"

namespace py = pybind11;
typedef LatticeBasedSproutingRule<3 > LatticeBasedSproutingRule3;
;
typedef ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > _std_vectorstd_shared_ptrVesselNode3_std_allocatorstd_shared_ptrVesselNode3;

class LatticeBasedSproutingRule3_Overloads : public LatticeBasedSproutingRule3{
    public:
    using LatticeBasedSproutingRule3::LatticeBasedSproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorstd_shared_ptrVesselNode3_std_allocatorstd_shared_ptrVesselNode3,
            LatticeBasedSproutingRule3,
            GetSprouts,
            rNodes);
    }
    void SetGridCalculator(::std::shared_ptr<GridCalculator<3> > pGrid) override {
        PYBIND11_OVERLOAD(
            void,
            LatticeBasedSproutingRule3,
            SetGridCalculator,
            pGrid);
    }

};
void register_LatticeBasedSproutingRule3_class(py::module &m){
py::class_<LatticeBasedSproutingRule3 , LatticeBasedSproutingRule3_Overloads   >(m, "LatticeBasedSproutingRule3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<LatticeBasedSproutingRule<3> >(*)()) &LatticeBasedSproutingRule3::Create, 
            " " )
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(LatticeBasedSproutingRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &LatticeBasedSproutingRule3::GetSprouts, 
            " " , py::arg("rNodes"))
        .def(
            "SetGridCalculator", 
            (void(LatticeBasedSproutingRule3::*)(::std::shared_ptr<GridCalculator<3> >)) &LatticeBasedSproutingRule3::SetGridCalculator, 
            " " , py::arg("pGrid"))
        .def(
            "SetTipExclusionRadius", 
            (void(LatticeBasedSproutingRule3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &LatticeBasedSproutingRule3::SetTipExclusionRadius, 
            " " , py::arg("tipExclusionRadius"))
    ;
}
