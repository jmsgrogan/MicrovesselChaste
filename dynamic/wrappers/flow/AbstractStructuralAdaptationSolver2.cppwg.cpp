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

#include "AbstractStructuralAdaptationSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractStructuralAdaptationSolver<2 > AbstractStructuralAdaptationSolver2;
;

class AbstractStructuralAdaptationSolver2_Overloads : public AbstractStructuralAdaptationSolver2{
    public:
    using AbstractStructuralAdaptationSolver2::AbstractStructuralAdaptationSolver;
    void Iterate() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractStructuralAdaptationSolver2,
            Iterate,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractStructuralAdaptationSolver2,
            Write,
            );
    }

};
void register_AbstractStructuralAdaptationSolver2_class(py::module &m){
py::class_<AbstractStructuralAdaptationSolver2 , AbstractStructuralAdaptationSolver2_Overloads   >(m, "AbstractStructuralAdaptationSolver2")
        .def(
            "GetTolerance", 
            (double(AbstractStructuralAdaptationSolver2::*)() const ) &AbstractStructuralAdaptationSolver2::GetTolerance, 
            " " )
        .def(
            "GetWriteOutput", 
            (bool(AbstractStructuralAdaptationSolver2::*)() const ) &AbstractStructuralAdaptationSolver2::GetWriteOutput, 
            " " )
        .def(
            "GetOutputFileName", 
            (::std::string(AbstractStructuralAdaptationSolver2::*)() const ) &AbstractStructuralAdaptationSolver2::GetOutputFileName, 
            " " )
        .def(
            "GetTimeIncrement", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractStructuralAdaptationSolver2::*)() const ) &AbstractStructuralAdaptationSolver2::GetTimeIncrement, 
            " " )
        .def(
            "Iterate", 
            (void(AbstractStructuralAdaptationSolver2::*)()) &AbstractStructuralAdaptationSolver2::Iterate, 
            " " )
        .def(
            "SetTolerance", 
            (void(AbstractStructuralAdaptationSolver2::*)(double)) &AbstractStructuralAdaptationSolver2::SetTolerance, 
            " " , py::arg("tolerance"))
        .def(
            "SetTimeIncrement", 
            (void(AbstractStructuralAdaptationSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractStructuralAdaptationSolver2::SetTimeIncrement, 
            " " , py::arg("timeIncrement"))
        .def(
            "SetMaxIterations", 
            (void(AbstractStructuralAdaptationSolver2::*)(unsigned int)) &AbstractStructuralAdaptationSolver2::SetMaxIterations, 
            " " , py::arg("iterations"))
        .def(
            "SetWriteOutput", 
            (void(AbstractStructuralAdaptationSolver2::*)(bool)) &AbstractStructuralAdaptationSolver2::SetWriteOutput, 
            " " , py::arg("writeFlag"))
        .def(
            "SetOutputFileName", 
            (void(AbstractStructuralAdaptationSolver2::*)(::std::string const &)) &AbstractStructuralAdaptationSolver2::SetOutputFileName, 
            " " , py::arg("rFilename"))
        .def(
            "SetVesselNetwork", 
            (void(AbstractStructuralAdaptationSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AbstractStructuralAdaptationSolver2::SetVesselNetwork, 
            " " , py::arg("pNetwork"))
        .def(
            "Solve", 
            (void(AbstractStructuralAdaptationSolver2::*)()) &AbstractStructuralAdaptationSolver2::Solve, 
            " " )
        .def(
            "Write", 
            (void(AbstractStructuralAdaptationSolver2::*)()) &AbstractStructuralAdaptationSolver2::Write, 
            " " )
    ;
}
