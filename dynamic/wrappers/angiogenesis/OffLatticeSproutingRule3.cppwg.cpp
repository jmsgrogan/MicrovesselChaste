#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "OffLatticeSproutingRule.hpp"

#include "OffLatticeSproutingRule3.cppwg.hpp"

namespace py = pybind11;
typedef OffLatticeSproutingRule<3 > OffLatticeSproutingRule3;
;
typedef ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > _std_vectorstd_shared_ptrVesselNode3_std_allocatorstd_shared_ptrVesselNode3;

class OffLatticeSproutingRule3_Overloads : public OffLatticeSproutingRule3{
    public:
    using OffLatticeSproutingRule3::OffLatticeSproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorstd_shared_ptrVesselNode3_std_allocatorstd_shared_ptrVesselNode3,
            OffLatticeSproutingRule3,
            GetSprouts,
            rNodes);
    }

};
void register_OffLatticeSproutingRule3_class(py::module &m){
py::class_<OffLatticeSproutingRule3 , OffLatticeSproutingRule3_Overloads   >(m, "OffLatticeSproutingRule3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<OffLatticeSproutingRule<3> >(*)()) &OffLatticeSproutingRule3::Create, 
            " "  )
        .def(
            "SetTipExclusionRadius", 
            (void(OffLatticeSproutingRule3::*)(::QLength)) &OffLatticeSproutingRule3::SetTipExclusionRadius, 
            " " , py::arg("exclusionRadius") )
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(OffLatticeSproutingRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &OffLatticeSproutingRule3::GetSprouts, 
            " " , py::arg("rNodes") )
    ;
}
