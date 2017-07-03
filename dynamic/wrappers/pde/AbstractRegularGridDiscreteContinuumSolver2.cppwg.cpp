#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"

#include "AbstractRegularGridDiscreteContinuumSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractRegularGridDiscreteContinuumSolver<2 > AbstractRegularGridDiscreteContinuumSolver2;
;

class AbstractRegularGridDiscreteContinuumSolver2_Overloads : public AbstractRegularGridDiscreteContinuumSolver2{
    public:
    using AbstractRegularGridDiscreteContinuumSolver2::AbstractRegularGridDiscreteContinuumSolver;
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            Setup,
            );
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            UpdateCellData,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateSolution(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            Write,
            );
    }

};
void register_AbstractRegularGridDiscreteContinuumSolver2_class(py::module &m){
py::class_<AbstractRegularGridDiscreteContinuumSolver2 , AbstractRegularGridDiscreteContinuumSolver2_Overloads   >(m, "AbstractRegularGridDiscreteContinuumSolver2")
        .def(
            "Setup", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::Setup, 
            " " )
        .def(
            "UpdateCellData", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::UpdateCellData, 
            " " )
        .def(
            "UpdateSolution", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > &)) &AbstractRegularGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData"))
        .def(
            "UpdateSolution", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > &)) &AbstractRegularGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData"))
        .def(
            "Update", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::Update, 
            " " )
        .def(
            "Solve", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::Solve, 
            " " )
        .def(
            "Write", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::Write, 
            " " )
    ;
}
