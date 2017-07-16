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
#include "AbstractMigrationRule.hpp"

#include "AbstractMigrationRule3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractMigrationRule<3 > AbstractMigrationRule3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<Vertex<3>, std::allocator<Vertex<3> > > _std_vector_lt_Vertex_lt_3_gt__std_allocator_lt_Vertex_lt_3_gt__gt__gt_;
typedef ::std::vector<int, std::allocator<int> > _std_vector_lt_int_std_allocator_lt_int_gt__gt_;

class AbstractMigrationRule3_Overloads : public AbstractMigrationRule3{
    public:
    using AbstractMigrationRule3::AbstractMigrationRule;
    ::std::vector<Vertex<3>, std::allocator<Vertex<3> > > GetDirections(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_Vertex_lt_3_gt__std_allocator_lt_Vertex_lt_3_gt__gt__gt_,
            AbstractMigrationRule3,
            GetDirections,
            rNodes);
    }
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_int_std_allocator_lt_int_gt__gt_,
            AbstractMigrationRule3,
            GetIndices,
            rNodes);
    }

};
void register_AbstractMigrationRule3_class(py::module &m){
py::class_<AbstractMigrationRule3 , AbstractMigrationRule3_Overloads , std::shared_ptr<AbstractMigrationRule3 >   >(m, "AbstractMigrationRule3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AbstractMigrationRule<3> >(*)()) &AbstractMigrationRule3::Create, 
            " "  )
        .def(
            "GetDirections", 
            (::std::vector<Vertex<3>, std::allocator<Vertex<3> > >(AbstractMigrationRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &AbstractMigrationRule3::GetDirections, 
            " " , py::arg("rNodes") )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(AbstractMigrationRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &AbstractMigrationRule3::GetIndices, 
            " " , py::arg("rNodes") )
        .def(
            "SetIsSprouting", 
            (void(AbstractMigrationRule3::*)(bool)) &AbstractMigrationRule3::SetIsSprouting, 
            " " , py::arg("isSprouting") = true )
        .def(
            "SetDiscreteContinuumSolver", 
            (void(AbstractMigrationRule3::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<3> >)) &AbstractMigrationRule3::SetDiscreteContinuumSolver, 
            " " , py::arg("pSolver") )
        .def(
            "SetNetwork", 
            (void(AbstractMigrationRule3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AbstractMigrationRule3::SetNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetBoundingDomain", 
            (void(AbstractMigrationRule3::*)(::std::shared_ptr<Part<3> >)) &AbstractMigrationRule3::SetBoundingDomain, 
            " " , py::arg("pPart") )
        .def(
            "SetGridCalculator", 
            (void(AbstractMigrationRule3::*)(::std::shared_ptr<GridCalculator<3> >)) &AbstractMigrationRule3::SetGridCalculator, 
            " " , py::arg("pGrid") )
        .def(
            "SetCellPopulation", 
            (void(AbstractMigrationRule3::*)(::std::shared_ptr<AbstractCellPopulation<3, 3> >)) &AbstractMigrationRule3::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetUseMooreNeighbourhood", 
            (void(AbstractMigrationRule3::*)(bool)) &AbstractMigrationRule3::SetUseMooreNeighbourhood, 
            " " , py::arg("useMooreNeighbourhood") )
    ;
}
