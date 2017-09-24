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
#include "SolutionDependentDiscreteSource.hpp"

#include "PythonObjectConverters.hpp"
#include "SolutionDependentDiscreteSource2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef SolutionDependentDiscreteSource<2 > SolutionDependentDiscreteSource2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;

class SolutionDependentDiscreteSource2_Overloads : public SolutionDependentDiscreteSource2{
    public:
    using SolutionDependentDiscreteSource2::SolutionDependentDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
            SolutionDependentDiscreteSource2,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
            SolutionDependentDiscreteSource2,
            GetLinearInUValues,
            );
    }

};
void register_SolutionDependentDiscreteSource2_class(py::module &m){
py::class_<SolutionDependentDiscreteSource2 , SolutionDependentDiscreteSource2_Overloads , std::shared_ptr<SolutionDependentDiscreteSource2 >  , DiscreteSource<2>  >(m, "SolutionDependentDiscreteSource2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SolutionDependentDiscreteSource<2> >(*)()) &SolutionDependentDiscreteSource2::Create, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(SolutionDependentDiscreteSource2::*)()) &SolutionDependentDiscreteSource2::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(SolutionDependentDiscreteSource2::*)()) &SolutionDependentDiscreteSource2::GetLinearInUValues, 
            " "  )
        .def(
            "SetSolution", 
            (void(SolutionDependentDiscreteSource2::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >)) &SolutionDependentDiscreteSource2::SetSolution, 
            " " , py::arg("solution") )
        .def(
            "SetConstantInUSinkRatePerSolutionQuantity", 
            (void(SolutionDependentDiscreteSource2::*)(::QRate)) &SolutionDependentDiscreteSource2::SetConstantInUSinkRatePerSolutionQuantity, 
            " " , py::arg("value") )
        .def(
            "SetLinearInUSinkRatePerSolutionQuantity", 
            (void(SolutionDependentDiscreteSource2::*)(::QRatePerConcentration)) &SolutionDependentDiscreteSource2::SetLinearInUSinkRatePerSolutionQuantity, 
            " " , py::arg("value") )
    ;
}
