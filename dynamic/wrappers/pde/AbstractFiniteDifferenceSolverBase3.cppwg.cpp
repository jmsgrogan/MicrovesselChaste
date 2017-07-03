#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractFiniteDifferenceSolverBase.hpp"

#include "AbstractFiniteDifferenceSolverBase3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractFiniteDifferenceSolverBase<3 > AbstractFiniteDifferenceSolverBase3;
;

class AbstractFiniteDifferenceSolverBase3_Overloads : public AbstractFiniteDifferenceSolverBase3{
    public:
    using AbstractFiniteDifferenceSolverBase3::AbstractFiniteDifferenceSolverBase;
    void AddDiscreteTermsToMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase3,
            AddDiscreteTermsToMatrix,
            );
    }
    void AddDiscreteTermsToRhs() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase3,
            AddDiscreteTermsToRhs,
            );
    }
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractFiniteDifferenceSolverBase3,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractFiniteDifferenceSolverBase3,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase3,
            Solve,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase3,
            Setup,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase3,
            Update,
            );
    }

};
void register_AbstractFiniteDifferenceSolverBase3_class(py::module &m){
py::class_<AbstractFiniteDifferenceSolverBase3 , AbstractFiniteDifferenceSolverBase3_Overloads   >(m, "AbstractFiniteDifferenceSolverBase3")
        .def(
            "AddDiscreteTermsToMatrix", 
            (void(AbstractFiniteDifferenceSolverBase3::*)()) &AbstractFiniteDifferenceSolverBase3::AddDiscreteTermsToMatrix, 
            " " )
        .def(
            "AddDiscreteTermsToRhs", 
            (void(AbstractFiniteDifferenceSolverBase3::*)()) &AbstractFiniteDifferenceSolverBase3::AddDiscreteTermsToRhs, 
            " " )
        .def(
            "AssembleMatrix", 
            (void(AbstractFiniteDifferenceSolverBase3::*)()) &AbstractFiniteDifferenceSolverBase3::AssembleMatrix, 
            " " )
        .def(
            "AssembleVector", 
            (void(AbstractFiniteDifferenceSolverBase3::*)()) &AbstractFiniteDifferenceSolverBase3::AssembleVector, 
            " " )
        .def(
            "GetRGBoundaryConditions", 
            (::std::shared_ptr<std::vector<std::pair<bool, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> >, std::allocator<std::pair<bool, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > > >(AbstractFiniteDifferenceSolverBase3::*)()) &AbstractFiniteDifferenceSolverBase3::GetRGBoundaryConditions, 
            " " )
        .def(
            "Solve", 
            (void(AbstractFiniteDifferenceSolverBase3::*)()) &AbstractFiniteDifferenceSolverBase3::Solve, 
            " " )
        .def(
            "Setup", 
            (void(AbstractFiniteDifferenceSolverBase3::*)()) &AbstractFiniteDifferenceSolverBase3::Setup, 
            " " )
        .def(
            "Update", 
            (void(AbstractFiniteDifferenceSolverBase3::*)()) &AbstractFiniteDifferenceSolverBase3::Update, 
            " " )
        .def(
            "UpdateBoundaryConditionsEachSolve", 
            (void(AbstractFiniteDifferenceSolverBase3::*)(bool)) &AbstractFiniteDifferenceSolverBase3::UpdateBoundaryConditionsEachSolve, 
            " " , py::arg("doUpdate"))
    ;
}
