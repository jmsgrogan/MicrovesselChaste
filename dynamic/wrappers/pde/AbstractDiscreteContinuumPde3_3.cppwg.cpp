#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractDiscreteContinuumPde.hpp"

#include "AbstractDiscreteContinuumPde3_3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumPde<3,3 > AbstractDiscreteContinuumPde3_3;
;

class AbstractDiscreteContinuumPde3_3_Overloads : public AbstractDiscreteContinuumPde3_3{
    public:
    using AbstractDiscreteContinuumPde3_3::AbstractDiscreteContinuumPde;
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumPde3_3,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumPde3_3_class(py::module &m){
py::class_<AbstractDiscreteContinuumPde3_3 , AbstractDiscreteContinuumPde3_3_Overloads   >(m, "AbstractDiscreteContinuumPde3_3")
        .def(py::init< >())
        .def(
            "AddDiscreteSource", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::std::shared_ptr<DiscreteSource<3> >)) &AbstractDiscreteContinuumPde3_3::AddDiscreteSource, 
            " " , py::arg("pDiscreteSource"))
        .def(
            "ComputeIsotropicDiffusionTerm", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(AbstractDiscreteContinuumPde3_3::*)()) &AbstractDiscreteContinuumPde3_3::ComputeIsotropicDiffusionTerm, 
            " " )
        .def(
            "GetDiscreteSources", 
            (::std::vector<std::shared_ptr<DiscreteSource<3> >, std::allocator<std::shared_ptr<DiscreteSource<3> > > >(AbstractDiscreteContinuumPde3_3::*)()) &AbstractDiscreteContinuumPde3_3::GetDiscreteSources, 
            " " )
        .def(
            "SetIsotropicDiffusionConstant", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<2, 1> >, boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<-1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractDiscreteContinuumPde3_3::SetIsotropicDiffusionConstant, 
            " " , py::arg("diffusivity"))
        .def(
            "SetReferenceConcentration", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<-3, 1> >, boost::units::list<boost::units::dim<boost::units::amount_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type> >, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractDiscreteContinuumPde3_3::SetReferenceConcentration, 
            " " , py::arg("referenceConcentration"))
        .def(
            "SetReferenceLength", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractDiscreteContinuumPde3_3::SetReferenceLength, 
            " " , py::arg("referenceLength"))
        .def(
            "SetReferenceTime", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::time_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &AbstractDiscreteContinuumPde3_3::SetReferenceTime, 
            " " , py::arg("referenceTime"))
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumPde3_3::*)()) &AbstractDiscreteContinuumPde3_3::UpdateDiscreteSourceStrengths, 
            " " )
    ;
}
