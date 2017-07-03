#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AngiogenesisSolver.hpp"

#include "AngiogenesisSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AngiogenesisSolver<2 > AngiogenesisSolver2;
;

class AngiogenesisSolver2_Overloads : public AngiogenesisSolver2{
    public:
    using AngiogenesisSolver2::AngiogenesisSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver2,
            Increment,
            );
    }
    void DoSprouting() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver2,
            DoSprouting,
            );
    }
    void DoAnastamosis() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver2,
            DoAnastamosis,
            );
    }
    void UpdateNodalPositions(bool sprouting) override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver2,
            UpdateNodalPositions,
            sprouting);
    }

};
void register_AngiogenesisSolver2_class(py::module &m){
py::class_<AngiogenesisSolver2 , AngiogenesisSolver2_Overloads   >(m, "AngiogenesisSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AngiogenesisSolver<2> >(*)()) &AngiogenesisSolver2::Create, 
            " " )
        .def(
            "Increment", 
            (void(AngiogenesisSolver2::*)()) &AngiogenesisSolver2::Increment, 
            " " )
        .def(
            "IsSproutingRuleSet", 
            (bool(AngiogenesisSolver2::*)()) &AngiogenesisSolver2::IsSproutingRuleSet, 
            " " )
        .def(
            "SetDoAnastomosis", 
            (void(AngiogenesisSolver2::*)(bool)) &AngiogenesisSolver2::SetDoAnastomosis, 
            " " , py::arg("doAnastomosis"))
        .def(
            "Run", 
            (void(AngiogenesisSolver2::*)(bool)) &AngiogenesisSolver2::Run, 
            " " , py::arg("writeOutput") = false)
        .def(
            "SetAnastamosisRadius", 
            (void(AngiogenesisSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AngiogenesisSolver2::SetAnastamosisRadius, 
            " " , py::arg("radius"))
        .def(
            "SetBoundingDomain", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<Part<2> >)) &AngiogenesisSolver2::SetBoundingDomain, 
            " " , py::arg("pDomain"))
        .def(
            "SetCellPopulation", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<AbstractCellPopulation<2, 2> >, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AngiogenesisSolver2::SetCellPopulation, 
            " " , py::arg("pCellPopulation"), py::arg("cellPopulationReferenceLength"))
        .def(
            "SetMigrationRule", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<AbstractMigrationRule<2> >)) &AngiogenesisSolver2::SetMigrationRule, 
            " " , py::arg("pMigrationRule"))
        .def(
            "SetOutputFileHandler", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<OutputFileHandler>)) &AngiogenesisSolver2::SetOutputFileHandler, 
            " " , py::arg("pHandler"))
        .def(
            "SetSproutingRule", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<AbstractSproutingRule<2> >)) &AngiogenesisSolver2::SetSproutingRule, 
            " " , py::arg("pSproutingRule"))
        .def(
            "SetVesselGridCalculator", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<GridCalculator<2> >)) &AngiogenesisSolver2::SetVesselGridCalculator, 
            " " , py::arg("pVesselGrid"))
        .def(
            "SetVesselNetwork", 
            (void(AngiogenesisSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AngiogenesisSolver2::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
    ;
}
