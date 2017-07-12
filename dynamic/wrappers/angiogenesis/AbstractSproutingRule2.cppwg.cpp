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
#include "AbstractSproutingRule.hpp"

#include "AbstractSproutingRule2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractSproutingRule<2 > AbstractSproutingRule2;
;
typedef ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > _std_vectorstd_shared_ptrVesselNode2_std_allocatorstd_shared_ptrVesselNode2;

class AbstractSproutingRule2_Overloads : public AbstractSproutingRule2{
    public:
    using AbstractSproutingRule2::AbstractSproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorstd_shared_ptrVesselNode2_std_allocatorstd_shared_ptrVesselNode2,
            AbstractSproutingRule2,
            GetSprouts,
            rNodes);
    }
    void SetGridCalculator(::std::shared_ptr<GridCalculator<2> > pGrid) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractSproutingRule2,
            SetGridCalculator,
            pGrid);
    }

};
void register_AbstractSproutingRule2_class(py::module &m){
py::class_<AbstractSproutingRule2 , AbstractSproutingRule2_Overloads   >(m, "AbstractSproutingRule2")
        .def(py::init< >())
        .def(
            "SetOnlySproutIfPerfused", 
            (void(AbstractSproutingRule2::*)(bool)) &AbstractSproutingRule2::SetOnlySproutIfPerfused, 
            " " , py::arg("onlySproutIfPerfused") )
        .def(
            "SetDiscreteContinuumSolver", 
            (void(AbstractSproutingRule2::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<2> >)) &AbstractSproutingRule2::SetDiscreteContinuumSolver, 
            " " , py::arg("pSolver") )
        .def(
            "SetVesselNetwork", 
            (void(AbstractSproutingRule2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AbstractSproutingRule2::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "SetSproutingProbability", 
            (void(AbstractSproutingRule2::*)(::QRate)) &AbstractSproutingRule2::SetSproutingProbability, 
            " " , py::arg("probability") )
        .def(
            "GetSproutingProbability", 
            (::QRate(AbstractSproutingRule2::*)()) &AbstractSproutingRule2::GetSproutingProbability, 
            " "  )
        .def(
            "SetVesselEndCutoff", 
            (void(AbstractSproutingRule2::*)(::QLength)) &AbstractSproutingRule2::SetVesselEndCutoff, 
            " " , py::arg("cutoff") )
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > >(AbstractSproutingRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &AbstractSproutingRule2::GetSprouts, 
            " " , py::arg("rNodes") )
        .def(
            "SetGridCalculator", 
            (void(AbstractSproutingRule2::*)(::std::shared_ptr<GridCalculator<2> >)) &AbstractSproutingRule2::SetGridCalculator, 
            " " , py::arg("pGrid") )
    ;
}
