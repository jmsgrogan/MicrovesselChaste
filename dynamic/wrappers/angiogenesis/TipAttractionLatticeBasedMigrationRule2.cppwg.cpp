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
#include "TipAttractionLatticeBasedMigrationRule.hpp"

#include "TipAttractionLatticeBasedMigrationRule2.cppwg.hpp"

namespace py = pybind11;
typedef TipAttractionLatticeBasedMigrationRule<2 > TipAttractionLatticeBasedMigrationRule2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<int, std::allocator<int> > _std_vector_lt_int_std_allocator_lt_int_gt__gt_;
typedef ::std::vector<double, std::allocator<double> > _std_vector_lt_double_std_allocator_lt_double_gt__gt_;

class TipAttractionLatticeBasedMigrationRule2_Overloads : public TipAttractionLatticeBasedMigrationRule2{
    public:
    using TipAttractionLatticeBasedMigrationRule2::TipAttractionLatticeBasedMigrationRule;
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_int_std_allocator_lt_int_gt__gt_,
            TipAttractionLatticeBasedMigrationRule2,
            GetIndices,
            rNodes);
    }

};
void register_TipAttractionLatticeBasedMigrationRule2_class(py::module &m){
py::class_<TipAttractionLatticeBasedMigrationRule2 , TipAttractionLatticeBasedMigrationRule2_Overloads , std::shared_ptr<TipAttractionLatticeBasedMigrationRule2 >  , LatticeBasedMigrationRule<2>  >(m, "TipAttractionLatticeBasedMigrationRule2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<TipAttractionLatticeBasedMigrationRule<2> >(*)()) &TipAttractionLatticeBasedMigrationRule2::Create, 
            " "  )
        .def(
            "GetIndices", 
            (::std::vector<int, std::allocator<int> >(TipAttractionLatticeBasedMigrationRule2::*)(::std::vector<std::shared_ptr<VesselNode<2> >, std::allocator<std::shared_ptr<VesselNode<2> > > > const &)) &TipAttractionLatticeBasedMigrationRule2::GetIndices, 
            " " , py::arg("rNodes") )
        .def(
            "SetCellChemotacticParameter", 
            (void(TipAttractionLatticeBasedMigrationRule2::*)(::QDiffusivityPerConcentration)) &TipAttractionLatticeBasedMigrationRule2::SetCellChemotacticParameter, 
            " " , py::arg("cellChemotacticParameter") )
        .def(
            "SetCellMotilityParameter", 
            (void(TipAttractionLatticeBasedMigrationRule2::*)(::QDiffusivity)) &TipAttractionLatticeBasedMigrationRule2::SetCellMotilityParameter, 
            " " , py::arg("cellMotility") )
        .def(
            "SetUseTipAttraction", 
            (void(TipAttractionLatticeBasedMigrationRule2::*)(bool)) &TipAttractionLatticeBasedMigrationRule2::SetUseTipAttraction, 
            " " , py::arg("useTipAttraction") )
    ;
}
