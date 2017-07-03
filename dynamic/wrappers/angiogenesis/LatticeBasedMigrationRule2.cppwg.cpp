#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "LatticeBasedMigrationRule.hpp"

#include "LatticeBasedMigrationRule2.cppwg.hpp"

namespace py = pybind11;
typedef LatticeBasedMigrationRule<2 > LatticeBasedMigrationRule2;
;
typedef ::std::vector<int, std::allocator<int> > _std_vectorint_std_allocatorint;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;

class LatticeBasedMigrationRule2_Overloads : public LatticeBasedMigrationRule2{
    public:
    using LatticeBasedMigrationRule2::LatticeBasedMigrationRule;
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorint_std_allocatorint,
            LatticeBasedMigrationRule2,
            GetIndices,
            rNodes);
    }
    ::std::vector<double, std::allocator<double> > GetNeighbourMovementProbabilities(::std::shared_ptr<VesselNode<2> > pNode, ::std::vector<unsigned int, std::allocator<unsigned int> > neighbourIndices, unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            LatticeBasedMigrationRule2,
            GetNeighbourMovementProbabilities,
            pNode, 
neighbourIndices, 
gridIndex);
    }
    int GetNeighbourMovementIndex(::std::vector<double, std::allocator<double> > movementProbabilities, ::std::vector<unsigned int, std::allocator<unsigned int> > neighbourIndices) override {
        PYBIND11_OVERLOAD(
            int,
            LatticeBasedMigrationRule2,
            GetNeighbourMovementIndex,
            movementProbabilities, 
neighbourIndices);
    }

};
void register_LatticeBasedMigrationRule2_class(py::module &m){
py::class_<LatticeBasedMigrationRule2 , LatticeBasedMigrationRule2_Overloads   >(m, "LatticeBasedMigrationRule2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<LatticeBasedMigrationRule<2> >(*)()) &LatticeBasedMigrationRule2::Create, 
            " " )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(LatticeBasedMigrationRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &LatticeBasedMigrationRule2::GetIndices, 
            " " , py::arg("rNodes"))
        .def(
            "SetMovementProbability", 
            (void(LatticeBasedMigrationRule2::*)(double)) &LatticeBasedMigrationRule2::SetMovementProbability, 
            " " , py::arg("movementProbability"))
    ;
}
