#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "vtkPolyData.h"
#include "OffLatticeSproutingRule.hpp"

#include "OffLatticeSproutingRule3.cppwg.hpp"

namespace py = pybind11;
typedef OffLatticeSproutingRule<3 > OffLatticeSproutingRule3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > _std_vector_lt_std_shared_ptr_lt_VesselNode_lt_3_gt__gt__std_allocator_lt_std_shared_ptr_lt_VesselNode_lt_3_gt__gt__gt__gt_;

class OffLatticeSproutingRule3_Overloads : public OffLatticeSproutingRule3{
    public:
    using OffLatticeSproutingRule3::OffLatticeSproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_std_shared_ptr_lt_VesselNode_lt_3_gt__gt__std_allocator_lt_std_shared_ptr_lt_VesselNode_lt_3_gt__gt__gt__gt_,
            OffLatticeSproutingRule3,
            GetSprouts,
            rNodes);
    }

};
void register_OffLatticeSproutingRule3_class(py::module &m){
py::class_<OffLatticeSproutingRule3 , OffLatticeSproutingRule3_Overloads , std::shared_ptr<OffLatticeSproutingRule3 >  , AbstractSproutingRule<3>  >(m, "OffLatticeSproutingRule3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<OffLatticeSproutingRule<3> >(*)()) &OffLatticeSproutingRule3::Create, 
            " "  )
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(OffLatticeSproutingRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &OffLatticeSproutingRule3::GetSprouts, 
            " " , py::arg("rNodes") )
        .def(
            "SetHalfMaxVegf", 
            (void(OffLatticeSproutingRule3::*)(::QConcentration)) &OffLatticeSproutingRule3::SetHalfMaxVegf, 
            " " , py::arg("halfMaxVegf") )
    ;
}
