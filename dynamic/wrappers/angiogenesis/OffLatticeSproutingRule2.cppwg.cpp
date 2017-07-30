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

#include "OffLatticeSproutingRule2.cppwg.hpp"

namespace py = pybind11;
typedef OffLatticeSproutingRule<2 > OffLatticeSproutingRule2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > _std_vector_lt_std_shared_ptr_lt_VesselNode_lt_2_gt__gt__std_allocator_lt_std_shared_ptr_lt_VesselNode_lt_2_gt__gt__gt__gt_;

class OffLatticeSproutingRule2_Overloads : public OffLatticeSproutingRule2{
    public:
    using OffLatticeSproutingRule2::OffLatticeSproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_std_shared_ptr_lt_VesselNode_lt_2_gt__gt__std_allocator_lt_std_shared_ptr_lt_VesselNode_lt_2_gt__gt__gt__gt_,
            OffLatticeSproutingRule2,
            GetSprouts,
            rNodes);
    }

};
void register_OffLatticeSproutingRule2_class(py::module &m){
py::class_<OffLatticeSproutingRule2 , OffLatticeSproutingRule2_Overloads , std::shared_ptr<OffLatticeSproutingRule2 >  , AbstractSproutingRule<2>  >(m, "OffLatticeSproutingRule2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<OffLatticeSproutingRule<2> >(*)()) &OffLatticeSproutingRule2::Create, 
            " "  )
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(OffLatticeSproutingRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &OffLatticeSproutingRule2::GetSprouts, 
            " " , py::arg("rNodes") )
        .def(
            "SetHalfMaxVegf", 
            (void(OffLatticeSproutingRule2::*)(::QConcentration)) &OffLatticeSproutingRule2::SetHalfMaxVegf, 
            " " , py::arg("halfMaxVegf") )
    ;
}
