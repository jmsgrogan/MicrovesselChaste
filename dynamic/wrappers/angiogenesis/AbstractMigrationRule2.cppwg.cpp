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

#include "PythonObjectConverters.hpp"
#include "AbstractMigrationRule2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractMigrationRule<2 > AbstractMigrationRule2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<Vertex<2>, std::allocator<Vertex<2> > > _std_vector_lt_Vertex_lt_2_gt__std_allocator_lt_Vertex_lt_2_gt__gt__gt_;
typedef ::std::vector<int, std::allocator<int> > _std_vector_lt_int_std_allocator_lt_int_gt__gt_;

class AbstractMigrationRule2_Overloads : public AbstractMigrationRule2{
    public:
    using AbstractMigrationRule2::AbstractMigrationRule;
    ::std::vector<Vertex<2>, std::allocator<Vertex<2> > > GetDirections(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_Vertex_lt_2_gt__std_allocator_lt_Vertex_lt_2_gt__gt__gt_,
            AbstractMigrationRule2,
            GetDirections,
            rNodes);
    }
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_int_std_allocator_lt_int_gt__gt_,
            AbstractMigrationRule2,
            GetIndices,
            rNodes);
    }

};
void register_AbstractMigrationRule2_class(py::module &m){
py::class_<AbstractMigrationRule2 , AbstractMigrationRule2_Overloads , std::shared_ptr<AbstractMigrationRule2 >   >(m, "AbstractMigrationRule2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<AbstractMigrationRule<2> >(*)()) &AbstractMigrationRule2::Create, 
            " "  )
        .def(
            "GetDirections", 
            (::std::vector<Vertex<2>, std::allocator<Vertex<2> > >(AbstractMigrationRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &AbstractMigrationRule2::GetDirections, 
            " " , py::arg("rNodes") )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(AbstractMigrationRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &AbstractMigrationRule2::GetIndices, 
            " " , py::arg("rNodes") )
        .def(
            "SetIsSprouting", 
            (void(AbstractMigrationRule2::*)(bool)) &AbstractMigrationRule2::SetIsSprouting, 
            " " , py::arg("isSprouting") = true )
        .def(
            "SetDiscreteContinuumSolver", 
            (void(AbstractMigrationRule2::*)(::std::shared_ptr<AbstractDiscreteContinuumSolver<2> >)) &AbstractMigrationRule2::SetDiscreteContinuumSolver, 
            " " , py::arg("pSolver") )
        .def(
            "SetNetwork", 
            (void(AbstractMigrationRule2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AbstractMigrationRule2::SetNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetBoundingDomain", 
            (void(AbstractMigrationRule2::*)(::std::shared_ptr<Part<2> >)) &AbstractMigrationRule2::SetBoundingDomain, 
            " " , py::arg("pPart") )
        .def(
            "SetGridCalculator", 
            (void(AbstractMigrationRule2::*)(::std::shared_ptr<GridCalculator<2> >)) &AbstractMigrationRule2::SetGridCalculator, 
            " " , py::arg("pGrid") )
        .def(
            "SetCellPopulation", 
            (void(AbstractMigrationRule2::*)(::std::shared_ptr<AbstractCellPopulation<2, 2> >)) &AbstractMigrationRule2::SetCellPopulation, 
            " " , py::arg("pCellPopulation") )
        .def(
            "SetUseMooreNeighbourhood", 
            (void(AbstractMigrationRule2::*)(bool)) &AbstractMigrationRule2::SetUseMooreNeighbourhood, 
            " " , py::arg("useMooreNeighbourhood") )
    ;
}
