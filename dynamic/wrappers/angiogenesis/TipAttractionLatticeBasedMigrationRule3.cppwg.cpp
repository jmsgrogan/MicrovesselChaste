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

#include "PythonObjectConverters.hpp"
#include "TipAttractionLatticeBasedMigrationRule3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef TipAttractionLatticeBasedMigrationRule<3 > TipAttractionLatticeBasedMigrationRule3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<int, std::allocator<int> > _std_vector_lt_int_std_allocator_lt_int_gt__gt_;
typedef ::std::vector<double, std::allocator<double> > _std_vector_lt_double_std_allocator_lt_double_gt__gt_;

class TipAttractionLatticeBasedMigrationRule3_Overloads : public TipAttractionLatticeBasedMigrationRule3{
    public:
    using TipAttractionLatticeBasedMigrationRule3::TipAttractionLatticeBasedMigrationRule;
    ::std::vector<int, std::allocator<int> > GetIndices(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_int_std_allocator_lt_int_gt__gt_,
            TipAttractionLatticeBasedMigrationRule3,
            GetIndices,
            rNodes);
    }

};
void register_TipAttractionLatticeBasedMigrationRule3_class(py::module &m){
py::class_<TipAttractionLatticeBasedMigrationRule3 , TipAttractionLatticeBasedMigrationRule3_Overloads , std::shared_ptr<TipAttractionLatticeBasedMigrationRule3 >  , LatticeBasedMigrationRule<3>  >(m, "TipAttractionLatticeBasedMigrationRule3")
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
