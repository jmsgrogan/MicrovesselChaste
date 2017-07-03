#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "ConstantHaematocritSolver.hpp"

#include "ConstantHaematocritSolver2.cppwg.hpp"

namespace py = pybind11;
typedef ConstantHaematocritSolver<2 > ConstantHaematocritSolver2;
;

class ConstantHaematocritSolver2_Overloads : public ConstantHaematocritSolver2{
    public:
    using ConstantHaematocritSolver2::ConstantHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ConstantHaematocritSolver2,
            Calculate,
            );
    }

};
void register_ConstantHaematocritSolver2_class(py::module &m){
py::class_<ConstantHaematocritSolver2 , ConstantHaematocritSolver2_Overloads   >(m, "ConstantHaematocritSolver2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ConstantHaematocritSolver<2> >(*)()) &ConstantHaematocritSolver2::Create, 
            " " )
        .def(
            "Calculate", 
            (void(ConstantHaematocritSolver2::*)()) &ConstantHaematocritSolver2::Calculate, 
            " " )
        .def(
            "SetHaematocrit", 
            (void(ConstantHaematocritSolver2::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &ConstantHaematocritSolver2::SetHaematocrit, 
            " " , py::arg("haematocrit"))
    ;
}
