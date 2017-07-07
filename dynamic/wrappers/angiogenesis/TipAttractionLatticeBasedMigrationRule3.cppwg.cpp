#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "TipAttractionLatticeBasedMigrationRule.hpp"

#include "TipAttractionLatticeBasedMigrationRule3.cppwg.hpp"

namespace py = pybind11;
typedef TipAttractionLatticeBasedMigrationRule<3 > TipAttractionLatticeBasedMigrationRule3;
;
typedef ::std::vector<int, std::allocator<int> > _std_vectorint_std_allocatorint;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;

class TipAttractionLatticeBasedMigrationRule3_Overloads : public TipAttractionLatticeBasedMigrationRule3{
    public:
    using TipAttractionLatticeBasedMigrationRule3::TipAttractionLatticeBasedMigrationRule;
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vectorint_std_allocatorint,
            TipAttractionLatticeBasedMigrationRule3,
            GetIndices,
            rNodes);
    }

};
void register_TipAttractionLatticeBasedMigrationRule3_class(py::module &m){
py::class_<TipAttractionLatticeBasedMigrationRule3 , TipAttractionLatticeBasedMigrationRule3_Overloads   >(m, "TipAttractionLatticeBasedMigrationRule3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<TipAttractionLatticeBasedMigrationRule<3> >(*)()) &TipAttractionLatticeBasedMigrationRule3::Create, 
            " "  )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(TipAttractionLatticeBasedMigrationRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &TipAttractionLatticeBasedMigrationRule3::GetIndices, 
            " " , py::arg("rNodes") )
        .def(
            "SetCellChemotacticParameter", 
            (void(TipAttractionLatticeBasedMigrationRule3::*)(::QDiffusivityPerConcentration)) &TipAttractionLatticeBasedMigrationRule3::SetCellChemotacticParameter, 
            " " , py::arg("cellChemotacticParameter") )
        .def(
            "SetCellMotilityParameter", 
            (void(TipAttractionLatticeBasedMigrationRule3::*)(::QDiffusivity)) &TipAttractionLatticeBasedMigrationRule3::SetCellMotilityParameter, 
            " " , py::arg("cellMotility") )
        .def(
            "SetUseTipAttraction", 
            (void(TipAttractionLatticeBasedMigrationRule3::*)(bool)) &TipAttractionLatticeBasedMigrationRule3::SetUseTipAttraction, 
            " " , py::arg("useTipAttraction") )
    ;
}
