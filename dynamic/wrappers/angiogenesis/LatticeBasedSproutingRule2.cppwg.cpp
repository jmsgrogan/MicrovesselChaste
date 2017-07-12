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
#include "LatticeBasedSproutingRule.hpp"

#include "LatticeBasedSproutingRule2.cppwg.hpp"

namespace py = pybind11;
typedef LatticeBasedSproutingRule<2 > LatticeBasedSproutingRule2;
;
typedef ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > _std_vectorstd_shared_ptrVesselNode2_std_allocatorstd_shared_ptrVesselNode2;

class LatticeBasedSproutingRule2_Overloads : public LatticeBasedSproutingRule2{
    public:
    using LatticeBasedSproutingRule2::LatticeBasedSproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorstd_shared_ptrVesselNode2_std_allocatorstd_shared_ptrVesselNode2,
            LatticeBasedSproutingRule2,
            GetSprouts,
            rNodes);
    }
    void SetGridCalculator(::std::shared_ptr<GridCalculator<2> > pGrid) override {
        PYBIND11_OVERLOAD(
            void,
            LatticeBasedSproutingRule2,
            SetGridCalculator,
            pGrid);
    }

};
void register_LatticeBasedSproutingRule2_class(py::module &m){
py::class_<LatticeBasedSproutingRule2 , LatticeBasedSproutingRule2_Overloads   >(m, "LatticeBasedSproutingRule2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<LatticeBasedSproutingRule<2> >(*)()) &LatticeBasedSproutingRule2::Create, 
            " "  )
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(LatticeBasedSproutingRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &LatticeBasedSproutingRule2::GetSprouts, 
            " " , py::arg("rNodes") )
        .def(
            "SetGridCalculator", 
            (void(LatticeBasedSproutingRule2::*)(::std::shared_ptr<GridCalculator<2> >)) &LatticeBasedSproutingRule2::SetGridCalculator, 
            " " , py::arg("pGrid") )
        .def(
            "SetTipExclusionRadius", 
            (void(LatticeBasedSproutingRule2::*)(::QLength)) &LatticeBasedSproutingRule2::SetTipExclusionRadius, 
            " " , py::arg("tipExclusionRadius") )
    ;
}
