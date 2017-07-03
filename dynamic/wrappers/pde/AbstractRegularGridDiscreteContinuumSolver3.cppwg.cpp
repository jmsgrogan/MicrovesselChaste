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

#include "AbstractRegularGridDiscreteContinuumSolver3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractRegularGridDiscreteContinuumSolver<3 > AbstractRegularGridDiscreteContinuumSolver3;
;

class AbstractRegularGridDiscreteContinuumSolver3_Overloads : public AbstractRegularGridDiscreteContinuumSolver3{
    public:
    using AbstractRegularGridDiscreteContinuumSolver3::AbstractRegularGridDiscreteContinuumSolver;
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            Setup,
            );
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            UpdateCellData,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void UpdateSolution(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            Write,
            );
    }

};
void register_AbstractRegularGridDiscreteContinuumSolver3_class(py::module &m){
py::class_<AbstractRegularGridDiscreteContinuumSolver3 , AbstractRegularGridDiscreteContinuumSolver3_Overloads   >(m, "AbstractRegularGridDiscreteContinuumSolver3")
        .def(
            "Setup", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::Setup, 
            " " )
        .def(
            "UpdateCellData", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::UpdateCellData, 
            " " )
        .def(
            "UpdateSolution", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)(::std::vector<double, std::allocator<double> > &)) &AbstractRegularGridDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData"))
        .def(
            "UpdateSolution", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)(::std::vector<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>, std::allocator<boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> > > &)) &AbstractRegularGridDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData"))
        .def(
            "Update", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::Update, 
            " " )
        .def(
            "Solve", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::Solve, 
            " " )
        .def(
            "Write", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::Write, 
            " " )
    ;
}
