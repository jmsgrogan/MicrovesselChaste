#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "Vessel.hpp"

#include "Vessel3.cppwg.hpp"

namespace py = pybind11;
typedef Vessel<3 > Vessel3;
;
typedef ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> _boost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class Vessel3_Overloads : public Vessel3{
    public:
    using Vessel3::Vessel;
    ::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> GetRadius() const  override {
        PYBIND11_OVERLOAD(
            _boost_units_quantityboost_units_unitboost_units_listboost_units_dimboost_units_length_base_dimension_boost_units_static_rational1_1_boost_units_dimensionless_type_boost_units_homogeneous_systemboost_units_listboost_units_si_meter_base_unit_boost_units_listboost_units_scaled_base_unitboost_units_cgs_gram_base_unit_boost_units_scale10_static_rational3_boost_units_listboost_units_si_second_base_unit_boost_units_listboost_units_si_ampere_base_unit_boost_units_listboost_units_si_kelvin_base_unit_boost_units_listboost_units_si_mole_base_unit_boost_units_listboost_units_si_candela_base_unit_boost_units_listboost_units_angle_radian_base_unit_boost_units_listboost_units_angle_steradian_base_unit_boost_units_dimensionless_type_void_double,
            Vessel3,
            GetRadius,
            );
    }
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            Vessel3,
            GetOutputData,
            );
    }
    void SetRadius(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double> radius) override {
        PYBIND11_OVERLOAD(
            void,
            Vessel3,
            SetRadius,
            radius);
    }

};
void register_Vessel3_class(py::module &m){
py::class_<Vessel3 , Vessel3_Overloads   >(m, "Vessel3")
        .def_static(
            "Create", 
            (::std::shared_ptr<Vessel<3> >(*)(::std::shared_ptr<VesselSegment<3> >)) &Vessel3::Create, 
            " " , py::arg("pSegment"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Vessel<3> >(*)(::std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >)) &Vessel3::Create, 
            " " , py::arg("segments"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Vessel<3> >(*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >)) &Vessel3::Create, 
            " " , py::arg("nodes"))
        .def_static(
            "Create", 
            (::std::shared_ptr<Vessel<3> >(*)(::std::shared_ptr<VesselNode<3> >, ::std::shared_ptr<VesselNode<3> >)) &Vessel3::Create, 
            " " , py::arg("pStartNode"), py::arg("pEndNode"))
        .def(
            "AddSegment", 
            (void(Vessel3::*)(::std::shared_ptr<VesselSegment<3> >)) &Vessel3::AddSegment, 
            " " , py::arg("pSegment"))
        .def(
            "AddSegments", 
            (void(Vessel3::*)(::std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >)) &Vessel3::AddSegments, 
            " " , py::arg("pSegments"))
        .def(
            "CopyDataFromExistingVessel", 
            (void(Vessel3::*)(::std::shared_ptr<Vessel<3> >)) &Vessel3::CopyDataFromExistingVessel, 
            " " , py::arg("pTargetVessel"))
        .def(
            "DivideSegment", 
            (::std::shared_ptr<VesselNode<3> >(Vessel3::*)(::DimensionalChastePoint<3> const &, double)) &Vessel3::DivideSegment, 
            " " , py::arg("rLocation"), py::arg("distanceTolerance") = 9.9999999999999995E-7)
        .def(
            "GetClosestEndNodeDistance", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(Vessel3::*)(::DimensionalChastePoint<3> const &)) &Vessel3::GetClosestEndNodeDistance, 
            " " , py::arg("rLocation"))
        .def(
            "GetDistance", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(Vessel3::*)(::DimensionalChastePoint<3> const &) const ) &Vessel3::GetDistance, 
            " " , py::arg("rLocation"))
        .def(
            "GetConnectedVessels", 
            (::std::vector<std::shared_ptr<Vessel<3> >, std::allocator<std::shared_ptr<Vessel<3> > > >(Vessel3::*)()) &Vessel3::GetConnectedVessels, 
            " " )
        .def(
            "GetEndNode", 
            (::std::shared_ptr<VesselNode<3> >(Vessel3::*)()) &Vessel3::GetEndNode, 
            " " )
        .def(
            "GetFlowProperties", 
            (::std::shared_ptr<VesselFlowProperties<3> >(Vessel3::*)() const ) &Vessel3::GetFlowProperties, 
            " " )
        .def(
            "GetNodeAtOppositeEnd", 
            (::std::shared_ptr<VesselNode<3> >(Vessel3::*)(::std::shared_ptr<VesselNode<3> >)) &Vessel3::GetNodeAtOppositeEnd, 
            " " , py::arg("pQueryNode"))
        .def(
            "GetLength", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(Vessel3::*)() const ) &Vessel3::GetLength, 
            " " )
        .def(
            "GetRadius", 
            (::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>(Vessel3::*)() const ) &Vessel3::GetRadius, 
            " " )
        .def(
            "GetMaturity", 
            (double(Vessel3::*)() const ) &Vessel3::GetMaturity, 
            " " )
        .def(
            "GetNode", 
            (::std::shared_ptr<VesselNode<3> >(Vessel3::*)(unsigned int)) &Vessel3::GetNode, 
            " " , py::arg("index"))
        .def(
            "GetNodes", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(Vessel3::*)()) &Vessel3::GetNodes, 
            " " )
        .def(
            "rGetNodes", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &(Vessel3::*)()) &Vessel3::rGetNodes, 
            " " )
        .def(
            "GetNumberOfNodes", 
            (unsigned int(Vessel3::*)()) &Vessel3::GetNumberOfNodes, 
            " " )
        .def(
            "GetNumberOfSegments", 
            (unsigned int(Vessel3::*)()) &Vessel3::GetNumberOfSegments, 
            " " )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(Vessel3::*)()) &Vessel3::GetOutputData, 
            " " )
        .def(
            "GetSegment", 
            (::std::shared_ptr<VesselSegment<3> >(Vessel3::*)(unsigned int)) &Vessel3::GetSegment, 
            " " , py::arg("index"))
        .def(
            "GetSegments", 
            (::std::vector<std::shared_ptr<VesselSegment<3> >, std::allocator<std::shared_ptr<VesselSegment<3> > > >(Vessel3::*)()) &Vessel3::GetSegments, 
            " " )
        .def(
            "GetStartNode", 
            (::std::shared_ptr<VesselNode<3> >(Vessel3::*)()) &Vessel3::GetStartNode, 
            " " )
        .def(
            "GetGlobalIndex", 
            (unsigned int(Vessel3::*)()) &Vessel3::GetGlobalIndex, 
            " " )
        .def(
            "GetLocalIndex", 
            (unsigned int(Vessel3::*)()) &Vessel3::GetLocalIndex, 
            " " )
        .def(
            "GetOwnerRank", 
            (unsigned int(Vessel3::*)()) &Vessel3::GetOwnerRank, 
            " " )
        .def(
            "IsHalo", 
            (bool(Vessel3::*)()) &Vessel3::IsHalo, 
            " " )
        .def(
            "HasHalo", 
            (bool(Vessel3::*)()) &Vessel3::HasHalo, 
            " " )
        .def(
            "GetOtherProcessorRank", 
            (unsigned int(Vessel3::*)()) &Vessel3::GetOtherProcessorRank, 
            " " )
        .def(
            "GetOtherProcessorLocalIndex", 
            (unsigned int(Vessel3::*)()) &Vessel3::GetOtherProcessorLocalIndex, 
            " " )
        .def(
            "IsConnectedTo", 
            (bool(Vessel3::*)(::std::shared_ptr<Vessel<3> >)) &Vessel3::IsConnectedTo, 
            " " , py::arg("pOtherVessel"))
        .def(
            "Remove", 
            (void(Vessel3::*)()) &Vessel3::Remove, 
            " " )
        .def(
            "RemoveSegments", 
            (void(Vessel3::*)(::SegmentLocation::Value)) &Vessel3::RemoveSegments, 
            " " , py::arg("location"))
        .def(
            "SetRadius", 
            (void(Vessel3::*)(::boost::units::quantity<boost::units::unit<boost::units::list<boost::units::dim<boost::units::length_base_dimension, boost::units::static_rational<1, 1> >, boost::units::dimensionless_type>, boost::units::homogeneous_system<boost::units::list<boost::units::si::meter_base_unit, boost::units::list<boost::units::scaled_base_unit<boost::units::cgs::gram_base_unit, boost::units::scale<10, static_rational<3> > >, boost::units::list<boost::units::si::second_base_unit, boost::units::list<boost::units::si::ampere_base_unit, boost::units::list<boost::units::si::kelvin_base_unit, boost::units::list<boost::units::si::mole_base_unit, boost::units::list<boost::units::si::candela_base_unit, boost::units::list<boost::units::angle::radian_base_unit, boost::units::list<boost::units::angle::steradian_base_unit, boost::units::dimensionless_type> > > > > > > > > >, void>, double>)) &Vessel3::SetRadius, 
            " " , py::arg("radius"))
        .def(
            "SetFlowProperties", 
            (void(Vessel3::*)(::VesselFlowProperties<3> const &)) &Vessel3::SetFlowProperties, 
            " " , py::arg("rFlowProperties"))
        .def(
            "SetGlobalIndex", 
            (void(Vessel3::*)(unsigned int)) &Vessel3::SetGlobalIndex, 
            " " , py::arg("index"))
        .def(
            "SetLocalIndex", 
            (void(Vessel3::*)(unsigned int)) &Vessel3::SetLocalIndex, 
            " " , py::arg("index"))
        .def(
            "SetOwnerRank", 
            (void(Vessel3::*)(unsigned int)) &Vessel3::SetOwnerRank, 
            " " , py::arg("rank"))
        .def(
            "SetIsHalo", 
            (void(Vessel3::*)(bool)) &Vessel3::SetIsHalo, 
            " " , py::arg("isHalo"))
        .def(
            "SetHasHalo", 
            (void(Vessel3::*)(bool)) &Vessel3::SetHasHalo, 
            " " , py::arg("hasHalo"))
        .def(
            "SetOtherProcessorRank", 
            (void(Vessel3::*)(unsigned int)) &Vessel3::SetOtherProcessorRank, 
            " " , py::arg("otherRank"))
        .def(
            "SetOtherProcessorLocalIndex", 
            (void(Vessel3::*)(unsigned int)) &Vessel3::SetOtherProcessorLocalIndex, 
            " " , py::arg("otherIndex"))
        .def(
            "UpdateNodes", 
            (void(Vessel3::*)()) &Vessel3::UpdateNodes, 
            " " )
    ;
}
