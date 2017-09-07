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

#include "PythonObjectConverters.hpp"
#include "AbstractSproutingRule3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AbstractSproutingRule<3 > AbstractSproutingRule3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > _std_vector_lt_std_shared_ptr_lt_VesselNode_lt_3_gt__gt__std_allocator_lt_std_shared_ptr_lt_VesselNode_lt_3_gt__gt__gt__gt_;

class AbstractSproutingRule3_Overloads : public AbstractSproutingRule3{
    public:
    using AbstractSproutingRule3::AbstractSproutingRule;
    ::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > GetSprouts(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD_PURE(
            _std_vector_lt_std_shared_ptr_lt_VesselNode_lt_3_gt__gt__std_allocator_lt_std_shared_ptr_lt_VesselNode_lt_3_gt__gt__gt__gt_,
            AbstractSproutingRule3,
            GetSprouts,
            rNodes);
    }
    void SetGridCalculator(::std::shared_ptr<GridCalculator<3> > pGrid) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractSproutingRule3,
            SetGridCalculator,
            pGrid);
    }

};
void register_AbstractSproutingRule3_class(py::module &m){
py::class_<AbstractSproutingRule3 , AbstractSproutingRule3_Overloads , std::shared_ptr<AbstractSproutingRule3 >   >(m, "AbstractSproutingRule3")
        .def(py::init< >())
        .def(
            "GetSproutingProbability", 
            (::QRate(AbstractSproutingRule3::*)()) &AbstractSproutingRule3::GetSproutingProbability, 
            " "  )
        .def(
            "GetSprouts", 
            (::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > >(AbstractSproutingRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &AbstractSproutingRule3::GetSprouts, 
            " " , py::arg("rNodes") )
        .def(
            "SetDiscreteContinuumSolver", 
            (void(AbstractSproutingRule3::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<3> >)) &AbstractSproutingRule3::SetDiscreteContinuumSolver, 
            " " , py::arg("pSolver") )
        .def(
            "SetGridCalculator", 
            (void(AbstractSproutingRule3::*)(::std::shared_ptr<GridCalculator<3> >)) &AbstractSproutingRule3::SetGridCalculator, 
            " " , py::arg("pGrid") )
        .def(
            "SetOnlySproutIfPerfused", 
            (void(AbstractSproutingRule3::*)(bool)) &AbstractSproutingRule3::SetOnlySproutIfPerfused, 
            " " , py::arg("onlySproutIfPerfused") )
        .def(
            "SetSproutingProbability", 
            (void(AbstractSproutingRule3::*)(::QRate)) &AbstractSproutingRule3::SetSproutingProbability, 
            " " , py::arg("probability") )
        .def(
            "SetVesselNetwork", 
            (void(AbstractSproutingRule3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AbstractSproutingRule3::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
        .def(
            "SetUseVesselEndCutoff", 
            (void(AbstractSproutingRule3::*)(bool)) &AbstractSproutingRule3::SetUseVesselEndCutoff, 
            " " , py::arg("useCutoff") )
        .def(
            "SetUseLateralInhibition", 
            (void(AbstractSproutingRule3::*)(bool)) &AbstractSproutingRule3::SetUseLateralInhibition, 
            " " , py::arg("useInhibition") )
    ;
}
