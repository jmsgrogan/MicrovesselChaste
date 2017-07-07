#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "CellPopulationMigrationRule.hpp"

#include "CellPopulationMigrationRule3.cppwg.hpp"

namespace py = pybind11;
typedef CellPopulationMigrationRule<3 > CellPopulationMigrationRule3;
;
typedef ::std::vector<int, std::allocator<int> > _std_vectorint_std_allocatorint;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;

class CellPopulationMigrationRule3_Overloads : public CellPopulationMigrationRule3{
    public:
    using CellPopulationMigrationRule3::CellPopulationMigrationRule;
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorint_std_allocatorint,
            CellPopulationMigrationRule3,
            GetIndices,
            rNodes);
    }
    ::std::vector<double, std::allocator<double> > GetNeighbourMovementProbabilities(::std::shared_ptr<VesselNode<3> > pNode, ::std::vector<unsigned int, std::allocator<unsigned int> > neighbourIndices, unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            CellPopulationMigrationRule3,
            GetNeighbourMovementProbabilities,
            pNode, 
neighbourIndices, 
gridIndex);
    }

};
void register_CellPopulationMigrationRule3_class(py::module &m){
py::class_<CellPopulationMigrationRule3 , CellPopulationMigrationRule3_Overloads   >(m, "CellPopulationMigrationRule3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CellPopulationMigrationRule<3> >(*)()) &CellPopulationMigrationRule3::Create, 
            " "  )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(CellPopulationMigrationRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &CellPopulationMigrationRule3::GetIndices, 
            " " , py::arg("rNodes") )
        .def(
            "SetVolumeFraction", 
            (void(CellPopulationMigrationRule3::*)(::boost::shared_ptr<AbstractCellMutationState>, double)) &CellPopulationMigrationRule3::SetVolumeFraction, 
            " " , py::arg("mutation_state"), py::arg("volume_fraction") )
        .def(
            "GetOccupyingVolumeFraction", 
            (double(CellPopulationMigrationRule3::*)(::boost::shared_ptr<AbstractCellMutationState>)) &CellPopulationMigrationRule3::GetOccupyingVolumeFraction, 
            " " , py::arg("mutation_state") )
    ;
}
