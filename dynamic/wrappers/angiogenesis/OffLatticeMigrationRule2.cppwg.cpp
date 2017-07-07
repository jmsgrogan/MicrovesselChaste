#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "OffLatticeMigrationRule.hpp"

#include "OffLatticeMigrationRule2.cppwg.hpp"

namespace py = pybind11;
typedef OffLatticeMigrationRule<2 > OffLatticeMigrationRule2;
;
typedef ::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > > _std_vectorDimensionalChastePoint2_std_allocatorDimensionalChastePoint2;

class OffLatticeMigrationRule2_Overloads : public OffLatticeMigrationRule2{
    public:
    using OffLatticeMigrationRule2::OffLatticeMigrationRule;
    ::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > > GetDirections(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorDimensionalChastePoint2_std_allocatorDimensionalChastePoint2,
            OffLatticeMigrationRule2,
            GetDirections,
            rNodes);
    }

};
void register_OffLatticeMigrationRule2_class(py::module &m){
py::class_<OffLatticeMigrationRule2 , OffLatticeMigrationRule2_Overloads   >(m, "OffLatticeMigrationRule2")
        .def(py::init< >())
        .def(
            "CalculateDomainDistanceMap", 
            (void(OffLatticeMigrationRule2::*)()) &OffLatticeMigrationRule2::CalculateDomainDistanceMap, 
            " "  )
        .def(
            "CalculateDomainDistanceMap", 
            (void(OffLatticeMigrationRule2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &OffLatticeMigrationRule2::CalculateDomainDistanceMap, 
            " " , py::arg("pGrid") )
        .def_static(
            "Create", 
            (::std::shared_ptr<OffLatticeMigrationRule<2> >(*)()) &OffLatticeMigrationRule2::Create, 
            " "  )
        .def(
            "GetDirections", 
            (::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >(OffLatticeMigrationRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &OffLatticeMigrationRule2::GetDirections, 
            " " , py::arg("rNodes") )
        .def(
            "GetDirectionsForSprouts", 
            (::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >(OffLatticeMigrationRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &OffLatticeMigrationRule2::GetDirectionsForSprouts, 
            " " , py::arg("rNodes") )
        .def(
            "SetSproutingVelocity", 
            (void(OffLatticeMigrationRule2::*)(::QVelocity)) &OffLatticeMigrationRule2::SetSproutingVelocity, 
            " " , py::arg("velocity") )
        .def(
            "SetChemotacticStrength", 
            (void(OffLatticeMigrationRule2::*)(double)) &OffLatticeMigrationRule2::SetChemotacticStrength, 
            " " , py::arg("strength") )
        .def(
            "SetAttractionStrength", 
            (void(OffLatticeMigrationRule2::*)(double)) &OffLatticeMigrationRule2::SetAttractionStrength, 
            " " , py::arg("strength") )
        .def(
            "SetNumGradientEvaluationDivisions", 
            (void(OffLatticeMigrationRule2::*)(unsigned int)) &OffLatticeMigrationRule2::SetNumGradientEvaluationDivisions, 
            " " , py::arg("numDivisions") )
        .def(
            "SetPersistenceAngleSdv", 
            (void(OffLatticeMigrationRule2::*)(double)) &OffLatticeMigrationRule2::SetPersistenceAngleSdv, 
            " " , py::arg("angle") )
    ;
}
