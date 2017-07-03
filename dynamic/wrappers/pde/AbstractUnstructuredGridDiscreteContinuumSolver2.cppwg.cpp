#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractUnstructuredGridDiscreteContinuumSolver.hpp"

#include "AbstractUnstructuredGridDiscreteContinuumSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractUnstructuredGridDiscreteContinuumSolver<2 > AbstractUnstructuredGridDiscreteContinuumSolver2;
;
typedef ::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > _std_vectorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double_std_allocatorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double;

class AbstractUnstructuredGridDiscreteContinuumSolver2_Overloads : public AbstractUnstructuredGridDiscreteContinuumSolver2{
    public:
    using AbstractUnstructuredGridDiscreteContinuumSolver2::AbstractUnstructuredGridDiscreteContinuumSolver;
    ::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > GetConcentrationsAtCentroids() override {
        PYBIND11_OVERLOAD(
            _std_vectorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double_std_allocatorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            GetConcentrationsAtCentroids,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            Setup,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateElementSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            UpdateElementSolution,
            rData);
    }
    void UpdateSolution(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            UpdateCellData,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            Write,
            );
    }

};
void register_AbstractUnstructuredGridDiscreteContinuumSolver2_class(py::module &m){
py::class_<AbstractUnstructuredGridDiscreteContinuumSolver2 , AbstractUnstructuredGridDiscreteContinuumSolver2_Overloads   >(m, "AbstractUnstructuredGridDiscreteContinuumSolver2")
        .def(
            "GetConcentrationsAtCentroids", 
            (::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > >(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::GetConcentrationsAtCentroids, 
            " " )
        .def(
            "Setup", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::Setup, 
            " " )
        .def(
            "UpdateSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData"))
        .def(
            "UpdateElementSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver2::UpdateElementSolution, 
            " " , py::arg("rData"))
        .def(
            "UpdateSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData"))
        .def(
            "UpdateCellData", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::UpdateCellData, 
            " " )
        .def(
            "Update", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::Update, 
            " " )
        .def(
            "Solve", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::Solve, 
            " " )
        .def(
            "Write", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::Write, 
            " " )
    ;
}
