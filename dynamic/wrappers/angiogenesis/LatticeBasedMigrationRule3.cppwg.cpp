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

#include "LatticeBasedMigrationRule3.cppwg.hpp"

namespace py = pybind11;
typedef LatticeBasedMigrationRule<3 > LatticeBasedMigrationRule3;
;
typedef ::std::vector<int, std::allocator<int> > _std_vectorint_std_allocatorint;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;

class LatticeBasedMigrationRule3_Overloads : public LatticeBasedMigrationRule3{
    public:
    using LatticeBasedMigrationRule3::LatticeBasedMigrationRule;
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorint_std_allocatorint,
            LatticeBasedMigrationRule3,
            GetIndices,
            rNodes);
    }
    ::std::vector<double, std::allocator<double> > GetNeighbourMovementProbabilities(::std::shared_ptr<VesselNode<3> > pNode, ::std::vector<unsigned int, std::allocator<unsigned int> > neighbourIndices, unsigned int gridIndex) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            LatticeBasedMigrationRule3,
            GetNeighbourMovementProbabilities,
            pNode, 
neighbourIndices, 
gridIndex);
    }
    int GetNeighbourMovementIndex(::std::vector<double, std::allocator<double> > movementProbabilities, ::std::vector<unsigned int, std::allocator<unsigned int> > neighbourIndices) override {
        PYBIND11_OVERLOAD(
            int,
            LatticeBasedMigrationRule3,
            GetNeighbourMovementIndex,
            movementProbabilities, 
neighbourIndices);
    }

};
void register_LatticeBasedMigrationRule3_class(py::module &m){
py::class_<LatticeBasedMigrationRule3 , LatticeBasedMigrationRule3_Overloads   >(m, "LatticeBasedMigrationRule3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<LatticeBasedMigrationRule<3> >(*)()) &LatticeBasedMigrationRule3::Create, 
            " "  )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(LatticeBasedMigrationRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &LatticeBasedMigrationRule3::GetIndices, 
            " " , py::arg("rNodes") )
        .def(
            "SetMovementProbability", 
            (void(LatticeBasedMigrationRule3::*)(double)) &LatticeBasedMigrationRule3::SetMovementProbability, 
            " " , py::arg("movementProbability") )
    ;
}
