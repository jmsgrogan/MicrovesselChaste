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

#include "AngiogenesisSolver3.cppwg.hpp"

namespace py = pybind11;
typedef AngiogenesisSolver<3 > AngiogenesisSolver3;
;

class AngiogenesisSolver3_Overloads : public AngiogenesisSolver3{
    public:
    using AngiogenesisSolver3::AngiogenesisSolver;
    void Increment() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver3,
            Increment,
            );
    }
    void DoSprouting() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver3,
            DoSprouting,
            );
    }
    void DoAnastamosis() override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver3,
            DoAnastamosis,
            );
    }
    void UpdateNodalPositions(bool sprouting) override {
        PYBIND11_OVERLOAD(
            void,
            AngiogenesisSolver3,
            UpdateNodalPositions,
            sprouting);
    }

};
void register_AngiogenesisSolver3_class(py::module &m){
py::class_<AngiogenesisSolver3 , AngiogenesisSolver3_Overloads   >(m, "AngiogenesisSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AngiogenesisSolver<3> >(*)()) &AngiogenesisSolver3::Create, 
            " " )
        .def(
            "Increment", 
            (void(AngiogenesisSolver3::*)()) &AngiogenesisSolver3::Increment, 
            " " )
        .def(
            "IsSproutingRuleSet", 
            (bool(AngiogenesisSolver3::*)()) &AngiogenesisSolver3::IsSproutingRuleSet, 
            " " )
        .def(
            "SetDoAnastomosis", 
            (void(AngiogenesisSolver3::*)(bool)) &AngiogenesisSolver3::SetDoAnastomosis, 
            " " , py::arg("doAnastomosis"))
        .def(
            "Run", 
            (void(AngiogenesisSolver3::*)(bool)) &AngiogenesisSolver3::Run, 
            " " , py::arg("writeOutput") = false)
        .def(
            "SetAnastamosisRadius", 
            (void(AngiogenesisSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AngiogenesisSolver3::SetAnastamosisRadius, 
            " " , py::arg("radius"))
        .def(
            "SetBoundingDomain", 
            (void(AngiogenesisSolver3::*)(::std::shared_ptr<Part<3> >)) &AngiogenesisSolver3::SetBoundingDomain, 
            " " , py::arg("pDomain"))
        .def(
            "SetCellPopulation", 
            (void(AngiogenesisSolver3::*)(::std::shared_ptr<AbstractCellPopulation<3, 3> >, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AngiogenesisSolver3::SetCellPopulation, 
            " " , py::arg("pCellPopulation"), py::arg("cellPopulationReferenceLength"))
        .def(
            "SetMigrationRule", 
            (void(AngiogenesisSolver3::*)(::std::shared_ptr<AbstractMigrationRule<3> >)) &AngiogenesisSolver3::SetMigrationRule, 
            " " , py::arg("pMigrationRule"))
        .def(
            "SetOutputFileHandler", 
            (void(AngiogenesisSolver3::*)(::std::shared_ptr<OutputFileHandler>)) &AngiogenesisSolver3::SetOutputFileHandler, 
            " " , py::arg("pHandler"))
        .def(
            "SetSproutingRule", 
            (void(AngiogenesisSolver3::*)(::std::shared_ptr<AbstractSproutingRule<3> >)) &AngiogenesisSolver3::SetSproutingRule, 
            " " , py::arg("pSproutingRule"))
        .def(
            "SetVesselGridCalculator", 
            (void(AngiogenesisSolver3::*)(::std::shared_ptr<GridCalculator<3> >)) &AngiogenesisSolver3::SetVesselGridCalculator, 
            " " , py::arg("pVesselGrid"))
        .def(
            "SetVesselNetwork", 
            (void(AngiogenesisSolver3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AngiogenesisSolver3::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
    ;
}
