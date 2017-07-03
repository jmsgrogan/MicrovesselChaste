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

#include "AbstractFiniteDifferenceSolverBase2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractFiniteDifferenceSolverBase<2 > AbstractFiniteDifferenceSolverBase2;
;

class AbstractFiniteDifferenceSolverBase2_Overloads : public AbstractFiniteDifferenceSolverBase2{
    public:
    using AbstractFiniteDifferenceSolverBase2::AbstractFiniteDifferenceSolverBase;
    void AddDiscreteTermsToMatrix() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            AddDiscreteTermsToMatrix,
            );
    }
    void AddDiscreteTermsToRhs() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            AddDiscreteTermsToRhs,
            );
    }
    void AssembleMatrix() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractFiniteDifferenceSolverBase2,
            AssembleMatrix,
            );
    }
    void AssembleVector() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractFiniteDifferenceSolverBase2,
            AssembleVector,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            Solve,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            Setup,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractFiniteDifferenceSolverBase2,
            Update,
            );
    }

};
void register_AbstractFiniteDifferenceSolverBase2_class(py::module &m){
py::class_<AbstractFiniteDifferenceSolverBase2 , AbstractFiniteDifferenceSolverBase2_Overloads   >(m, "AbstractFiniteDifferenceSolverBase2")
        .def(
            "AddDiscreteTermsToMatrix", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::AddDiscreteTermsToMatrix, 
            " " )
        .def(
            "AddDiscreteTermsToRhs", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::AddDiscreteTermsToRhs, 
            " " )
        .def(
            "AssembleMatrix", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::AssembleMatrix, 
            " " )
        .def(
            "AssembleVector", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::AssembleVector, 
            " " )
        .def(
            "GetRGBoundaryConditions", 
            (::std::shared_ptr<std::vector<std::pair<bool, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> >, std::allocator<std::pair<bool, boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > > >(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::GetRGBoundaryConditions, 
            " " )
        .def(
            "Solve", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::Solve, 
            " " )
        .def(
            "Setup", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::Setup, 
            " " )
        .def(
            "Update", 
            (void(AbstractFiniteDifferenceSolverBase2::*)()) &AbstractFiniteDifferenceSolverBase2::Update, 
            " " )
        .def(
            "UpdateBoundaryConditionsEachSolve", 
            (void(AbstractFiniteDifferenceSolverBase2::*)(bool)) &AbstractFiniteDifferenceSolverBase2::UpdateBoundaryConditionsEachSolve, 
            " " , py::arg("doUpdate"))
    ;
}
