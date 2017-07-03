#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractSproutingRule.hpp"

#include "AbstractSproutingRule3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractSproutingRule<3 > AbstractSproutingRule3;
;
typedef ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > _std_vectorstd_shared_ptrVesselNode3_std_allocatorstd_shared_ptrVesselNode3;

class AbstractSproutingRule3_Overloads : public AbstractSproutingRule3{
    public:
    using AbstractSproutingRule3::AbstractSproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorstd_shared_ptrVesselNode3_std_allocatorstd_shared_ptrVesselNode3,
            AbstractSproutingRule3,
            GetSprouts,
            rNodes);
    }
    void SetGridCalculator(::std::shared_ptr<GridCalculator<3> > pGrid) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractSproutingRule3,
            SetGridCalculator,
            pGrid);
    }

};
void register_AbstractSproutingRule3_class(py::module &m){
py::class_<AbstractSproutingRule3 , AbstractSproutingRule3_Overloads   >(m, "AbstractSproutingRule3")
        .def(py::init< >())
        .def(
            "SetOnlySproutIfPerfused", 
            (void(AbstractSproutingRule3::*)(bool)) &AbstractSproutingRule3::SetOnlySproutIfPerfused, 
            " " , py::arg("onlySproutIfPerfused"))
        .def(
            "SetDiscreteContinuumSolver", 
            (void(AbstractSproutingRule3::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<3> >)) &AbstractSproutingRule3::SetDiscreteContinuumSolver, 
            " " , py::arg("pSolver"))
        .def(
            "SetVesselNetwork", 
            (void(AbstractSproutingRule3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AbstractSproutingRule3::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork"))
        .def(
            "SetSproutingProbability", 
            (void(AbstractSproutingRule3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractSproutingRule3::SetSproutingProbability, 
            " " , py::arg("probability"))
        .def(
            "GetSproutingProbability", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractSproutingRule3::*)()) &AbstractSproutingRule3::GetSproutingProbability, 
            " " )
        .def(
            "SetVesselEndCutoff", 
            (void(AbstractSproutingRule3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractSproutingRule3::SetVesselEndCutoff, 
            " " , py::arg("cutoff"))
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(AbstractSproutingRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &AbstractSproutingRule3::GetSprouts, 
            " " , py::arg("rNodes"))
        .def(
            "SetGridCalculator", 
            (void(AbstractSproutingRule3::*)(::std::shared_ptr<GridCalculator<3> >)) &AbstractSproutingRule3::SetGridCalculator, 
            " " , py::arg("pGrid"))
    ;
}
