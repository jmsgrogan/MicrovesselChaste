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
#include "OffLatticeMigrationRule.hpp"

#include "PythonObjectConverters.hpp"
#include "OffLatticeMigrationRule3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef OffLatticeMigrationRule<3 > OffLatticeMigrationRule3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<Vertex<3>, std::allocator<Vertex<3> > > _std_vector_lt_Vertex_lt_3_gt__std_allocator_lt_Vertex_lt_3_gt__gt__gt_;

class OffLatticeMigrationRule3_Overloads : public OffLatticeMigrationRule3{
    public:
    using OffLatticeMigrationRule3::OffLatticeMigrationRule;
    ::std::vector<Vertex<3>, std::allocator<Vertex<3> > > GetDirections(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const & rNodes) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_Vertex_lt_3_gt__std_allocator_lt_Vertex_lt_3_gt__gt__gt_,
            OffLatticeMigrationRule3,
            GetDirections,
            rNodes);
    }

};
void register_OffLatticeMigrationRule3_class(py::module &m){
py::class_<OffLatticeMigrationRule3 , OffLatticeMigrationRule3_Overloads , std::shared_ptr<OffLatticeMigrationRule3 >  , AbstractMigrationRule<3>  >(m, "OffLatticeMigrationRule3")
        .def(py::init< >())
        .def(
            "CalculateDomainDistanceMap", 
            (void(OffLatticeMigrationRule3::*)()) &OffLatticeMigrationRule3::CalculateDomainDistanceMap, 
            " "  )
        .def(
            "CalculateDomainDistanceMap", 
            (void(OffLatticeMigrationRule3::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >)) &OffLatticeMigrationRule3::CalculateDomainDistanceMap, 
            " " , py::arg("pGrid") )
        .def_static(
            "Create", 
            (::std::shared_ptr<OffLatticeMigrationRule<3> >(*)()) &OffLatticeMigrationRule3::Create, 
            " "  )
        .def(
            "GetDirections", 
            (::std::vector<Vertex<3>, std::allocator<Vertex<3> > >(OffLatticeMigrationRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &OffLatticeMigrationRule3::GetDirections, 
            " " , py::arg("rNodes") )
        .def(
            "GetDirectionsForSprouts", 
            (::std::vector<Vertex<3>, std::allocator<Vertex<3> > >(OffLatticeMigrationRule3::*)(::std::vector<std::shared_ptr<VesselNode<3> >, std::allocator<std::shared_ptr<VesselNode<3> > > > const &)) &OffLatticeMigrationRule3::GetDirectionsForSprouts, 
            " " , py::arg("rNodes") )
        .def(
            "SetSproutingVelocity", 
            (void(OffLatticeMigrationRule3::*)(::QVelocity)) &OffLatticeMigrationRule3::SetSproutingVelocity, 
            " " , py::arg("velocity") )
        .def(
            "SetChemotacticStrength", 
            (void(OffLatticeMigrationRule3::*)(double)) &OffLatticeMigrationRule3::SetChemotacticStrength, 
            " " , py::arg("strength") )
        .def(
            "SetAttractionStrength", 
            (void(OffLatticeMigrationRule3::*)(double)) &OffLatticeMigrationRule3::SetAttractionStrength, 
            " " , py::arg("strength") )
        .def(
            "SetNumGradientEvaluationDivisions", 
            (void(OffLatticeMigrationRule3::*)(unsigned int)) &OffLatticeMigrationRule3::SetNumGradientEvaluationDivisions, 
            " " , py::arg("numDivisions") )
        .def(
            "SetPersistenceAngleSdv", 
            (void(OffLatticeMigrationRule3::*)(double)) &OffLatticeMigrationRule3::SetPersistenceAngleSdv, 
            " " , py::arg("angle") )
    ;
}
