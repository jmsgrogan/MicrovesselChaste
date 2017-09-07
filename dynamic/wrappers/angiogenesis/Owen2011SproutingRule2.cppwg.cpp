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
#include "Owen2011SproutingRule.hpp"

#include "PythonObjectConverters.hpp"
#include "Owen2011SproutingRule2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef Owen2011SproutingRule<2 > Owen2011SproutingRule2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > _std_vector_lt_std_shared_ptr_lt_VesselNode_lt_2_gt__gt__std_allocator_lt_std_shared_ptr_lt_VesselNode_lt_2_gt__gt__gt__gt_;

class Owen2011SproutingRule2_Overloads : public Owen2011SproutingRule2{
    public:
    using Owen2011SproutingRule2::Owen2011SproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_std_shared_ptr_lt_VesselNode_lt_2_gt__gt__std_allocator_lt_std_shared_ptr_lt_VesselNode_lt_2_gt__gt__gt__gt_,
            Owen2011SproutingRule2,
            GetSprouts,
            rNodes);
    }

};
void register_Owen2011SproutingRule2_class(py::module &m){
py::class_<Owen2011SproutingRule2 , Owen2011SproutingRule2_Overloads , std::shared_ptr<Owen2011SproutingRule2 >  , LatticeBasedSproutingRule<2>  >(m, "Owen2011SproutingRule2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<Owen2011SproutingRule<2> >(*)()) &Owen2011SproutingRule2::Create, 
            " "  )
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(Owen2011SproutingRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &Owen2011SproutingRule2::GetSprouts, 
            " " , py::arg("rNodes") )
        .def(
            "SetHalfMaxVegf", 
            (void(Owen2011SproutingRule2::*)(::QConcentration)) &Owen2011SproutingRule2::SetHalfMaxVegf, 
            " " , py::arg("halfMaxVegf") )
    ;
}
