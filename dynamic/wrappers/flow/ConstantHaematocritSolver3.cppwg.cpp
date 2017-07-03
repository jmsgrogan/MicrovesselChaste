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

#include "ConstantHaematocritSolver3.cppwg.hpp"

namespace py = pybind11;
typedef ConstantHaematocritSolver<3 > ConstantHaematocritSolver3;
;

class ConstantHaematocritSolver3_Overloads : public ConstantHaematocritSolver3{
    public:
    using ConstantHaematocritSolver3::ConstantHaematocritSolver;
    void Calculate() override {
        PYBIND11_OVERLOAD(
            void,
            ConstantHaematocritSolver3,
            Calculate,
            );
    }

};
void register_ConstantHaematocritSolver3_class(py::module &m){
py::class_<ConstantHaematocritSolver3 , ConstantHaematocritSolver3_Overloads   >(m, "ConstantHaematocritSolver3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<ConstantHaematocritSolver<3> >(*)()) &ConstantHaematocritSolver3::Create, 
            " " )
        .def(
            "Calculate", 
            (void(ConstantHaematocritSolver3::*)()) &ConstantHaematocritSolver3::Calculate, 
            " " )
        .def(
            "SetHaematocrit", 
            (void(ConstantHaematocritSolver3::*)(::boost::units::quantity<boost::units::unit<boost::units::dimensionless_type, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &ConstantHaematocritSolver3::SetHaematocrit, 
            " " , py::arg("haematocrit"))
    ;
}
