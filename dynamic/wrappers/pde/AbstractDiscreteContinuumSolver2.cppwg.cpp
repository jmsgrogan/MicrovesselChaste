#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"

#include "AbstractDiscreteContinuumSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumSolver<2 > AbstractDiscreteContinuumSolver2;
;
typedef ::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > _std_vectorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double_std_allocatorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double;
typedef ::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > _std_vectorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double_std_allocatorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double;
typedef ::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > _std_vectorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double_std_allocatorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > > _std_vectorboost_numeric_ublas_c_vectordouble_3_std_allocatorboost_numeric_ublas_c_vectordouble_3;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::vtkSmartPointer<vtkDataSet> _vtkSmartPointervtkDataSet;

class AbstractDiscreteContinuumSolver2_Overloads : public AbstractDiscreteContinuumSolver2{
    public:
    using AbstractDiscreteContinuumSolver2::AbstractDiscreteContinuumSolver;
    ::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > GetConcentrations() override {
        PYBIND11_OVERLOAD(
            _std_vectorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double_std_allocatorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double,
            AbstractDiscreteContinuumSolver2,
            GetConcentrations,
            );
    }
    ::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > GetConcentrations(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> > pGrid) override {
        PYBIND11_OVERLOAD(
            _std_vectorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double_std_allocatorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double,
            AbstractDiscreteContinuumSolver2,
            GetConcentrations,
            pGrid);
    }
    ::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > GetConcentrations(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double_std_allocatorboost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational-3_1_boost_units_listboost_units_dimboost_units_amount_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double,
            AbstractDiscreteContinuumSolver2,
            GetConcentrations,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolution() override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver2,
            GetSolution,
            );
    }
    ::std::vector<double, std::allocator<double> > GetSolution(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver2,
            GetSolution,
            pSamplePoints);
    }
    ::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > > GetSolutionGradients(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectorboost_numeric_ublas_c_vectordouble_3_std_allocatorboost_numeric_ublas_c_vectordouble_3,
            AbstractDiscreteContinuumSolver2,
            GetSolutionGradients,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolutionP(::vtkPoints * pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver2,
            GetSolutionP,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolution(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> > pGrid) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver2,
            GetSolution,
            pGrid);
    }
    ::vtkSmartPointer<vtkDataSet> GetVtkSolution() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointervtkDataSet,
            AbstractDiscreteContinuumSolver2,
            GetVtkSolution,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            Setup,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            Solve,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            Update,
            );
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            UpdateCellData,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateSolution(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void Write() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            Write,
            );
    }

};
void register_AbstractDiscreteContinuumSolver2_class(py::module &m){
py::class_<AbstractDiscreteContinuumSolver2 , AbstractDiscreteContinuumSolver2_Overloads   >(m, "AbstractDiscreteContinuumSolver2")
        .def(
            "AddBoundaryCondition", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<DiscreteContinuumBoundaryCondition<2> >)) &AbstractDiscreteContinuumSolver2::AddBoundaryCondition, 
            " " , py::arg("pBoundaryCondition"))
        .def(
            "GetConcentrations", 
            (::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > >(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetConcentrations, 
            " " )
        .def(
            "GetConcentrations", 
            (::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > >(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &AbstractDiscreteContinuumSolver2::GetConcentrations, 
            " " , py::arg("pGrid"))
        .def(
            "GetConcentrations", 
            (::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > >(AbstractDiscreteContinuumSolver2::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver2::GetConcentrations, 
            " " , py::arg("pSamplePoints"))
        .def(
            "GetDensityMap", 
            (::std::shared_ptr<DensityMap<2> >(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetDensityMap, 
            " " )
        .def(
            "GetLabel", 
            (::std::string const &(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetLabel, 
            " " )
        .def(
            "GetPde", 
            (::std::shared_ptr<AbstractDiscreteContinuumPde<2, 2> >(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetPde, 
            " " )
        .def(
            "GetReferenceLength", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetReferenceLength, 
            " " )
        .def(
            "GetReferenceConcentration", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetReferenceConcentration, 
            " " )
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetSolution, 
            " " )
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver2::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver2::GetSolution, 
            " " , py::arg("pSamplePoints"))
        .def(
            "GetSolutionGradients", 
            (::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > >(AbstractDiscreteContinuumSolver2::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver2::GetSolutionGradients, 
            " " , py::arg("pSamplePoints"))
        .def(
            "GetSolutionP", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver2::*)(::vtkPoints *)) &AbstractDiscreteContinuumSolver2::GetSolutionP, 
            " " , py::arg("pSamplePoints"))
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &AbstractDiscreteContinuumSolver2::GetSolution, 
            " " , py::arg("pGrid"))
        .def(
            "GetVtkSolution", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetVtkSolution, 
            " " )
        .def(
            "SetFileHandler", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<OutputFileHandler>)) &AbstractDiscreteContinuumSolver2::SetFileHandler, 
            " " , py::arg("pOutputFileHandler"))
        .def(
            "SetFileName", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::string const &)) &AbstractDiscreteContinuumSolver2::SetFileName, 
            " " , py::arg("rFilename"))
        .def(
            "SetLabel", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::string const &)) &AbstractDiscreteContinuumSolver2::SetLabel, 
            " " , py::arg("rLabel"))
        .def(
            "SetPde", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumPde<2, 2> >)) &AbstractDiscreteContinuumSolver2::SetPde, 
            " " , py::arg("pPde"))
        .def(
            "Setup", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::Setup, 
            " " )
        .def(
            "SetReferenceConcentration", 
            (void(AbstractDiscreteContinuumSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractDiscreteContinuumSolver2::SetReferenceConcentration, 
            " " , py::arg("referenceConcentration"))
        .def(
            "SetWriteSolution", 
            (void(AbstractDiscreteContinuumSolver2::*)(bool)) &AbstractDiscreteContinuumSolver2::SetWriteSolution, 
            " " , py::arg("write") = true)
        .def(
            "SetGrid", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &AbstractDiscreteContinuumSolver2::SetGrid, 
            " " , py::arg("pGrid"))
        .def(
            "SetDensityMap", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<DensityMap<2> >)) &AbstractDiscreteContinuumSolver2::SetDensityMap, 
            " " , py::arg("pDensityMap"))
        .def(
            "SetVesselNetwork", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AbstractDiscreteContinuumSolver2::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "SetCellPopulation", 
            (void(AbstractDiscreteContinuumSolver2::*)(::AbstractCellPopulation<2, 2> &, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractDiscreteContinuumSolver2::SetCellPopulation, 
            " " , py::arg("rCellPopulation"), py::arg("cellPopulationReferenceLength"), py::arg("cellPopulationReferenceConcentration"))
        .def(
            "Solve", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::Solve, 
            " " )
        .def(
            "Update", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::Update, 
            " " )
        .def(
            "UpdateCellData", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::UpdateCellData, 
            " " )
        .def(
            "UpdateSolution", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData"))
        .def(
            "UpdateSolution", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > const &)) &AbstractDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData"))
        .def(
            "Write", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::Write, 
            " " )
    ;
}
