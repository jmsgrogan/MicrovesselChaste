#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractStructuralAdaptationSolver.hpp"

#include "AbstractStructuralAdaptationSolver3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractStructuralAdaptationSolver<3 > AbstractStructuralAdaptationSolver3;
;

class AbstractStructuralAdaptationSolver3_Overloads : public AbstractStructuralAdaptationSolver3{
    public:
    using AbstractStructuralAdaptationSolver3::AbstractStructuralAdaptationSolver;
    void Iterate() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractStructuralAdaptationSolver3,
            Iterate,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractStructuralAdaptationSolver3,
            Write,
            );
    }

};
void register_AbstractStructuralAdaptationSolver3_class(py::module &m){
py::class_<AbstractStructuralAdaptationSolver3 , AbstractStructuralAdaptationSolver3_Overloads   >(m, "AbstractStructuralAdaptationSolver3")
        .def(
            "GetTolerance", 
            (double(AbstractStructuralAdaptationSolver3::*)() const ) &AbstractStructuralAdaptationSolver3::GetTolerance, 
            " " )
        .def(
            "GetWriteOutput", 
            (bool(AbstractStructuralAdaptationSolver3::*)() const ) &AbstractStructuralAdaptationSolver3::GetWriteOutput, 
            " " )
        .def(
            "GetOutputFileName", 
            (::std::string(AbstractStructuralAdaptationSolver3::*)() const ) &AbstractStructuralAdaptationSolver3::GetOutputFileName, 
            " " )
        .def(
            "GetTimeIncrement", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractStructuralAdaptationSolver3::*)() const ) &AbstractStructuralAdaptationSolver3::GetTimeIncrement, 
            " " )
        .def(
            "Iterate", 
            (void(AbstractStructuralAdaptationSolver3::*)()) &AbstractStructuralAdaptationSolver3::Iterate, 
            " " )
        .def(
            "SetTolerance", 
            (void(AbstractStructuralAdaptationSolver3::*)(double)) &AbstractStructuralAdaptationSolver3::SetTolerance, 
            " " , py::arg("tolerance"))
        .def(
            "SetTimeIncrement", 
            (void(AbstractStructuralAdaptationSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractStructuralAdaptationSolver3::SetTimeIncrement, 
            " " , py::arg("timeIncrement"))
        .def(
            "SetMaxIterations", 
            (void(AbstractStructuralAdaptationSolver3::*)(unsigned int)) &AbstractStructuralAdaptationSolver3::SetMaxIterations, 
            " " , py::arg("iterations"))
        .def(
            "SetWriteOutput", 
            (void(AbstractStructuralAdaptationSolver3::*)(bool)) &AbstractStructuralAdaptationSolver3::SetWriteOutput, 
            " " , py::arg("writeFlag"))
        .def(
            "SetOutputFileName", 
            (void(AbstractStructuralAdaptationSolver3::*)(::std::string const &)) &AbstractStructuralAdaptationSolver3::SetOutputFileName, 
            " " , py::arg("rFilename"))
        .def(
            "SetVesselNetwork", 
            (void(AbstractStructuralAdaptationSolver3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AbstractStructuralAdaptationSolver3::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "Solve", 
            (void(AbstractStructuralAdaptationSolver3::*)()) &AbstractStructuralAdaptationSolver3::Solve, 
            " " )
        .def(
            "Write", 
            (void(AbstractStructuralAdaptationSolver3::*)()) &AbstractStructuralAdaptationSolver3::Write, 
            " " )
    ;
}
