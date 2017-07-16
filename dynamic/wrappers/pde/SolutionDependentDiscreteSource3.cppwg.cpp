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

#include "SolutionDependentDiscreteSource3.cppwg.hpp"

namespace py = pybind11;
typedef SolutionDependentDiscreteSource<3 > SolutionDependentDiscreteSource3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;

class SolutionDependentDiscreteSource3_Overloads : public SolutionDependentDiscreteSource3{
    public:
    using SolutionDependentDiscreteSource3::SolutionDependentDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
            SolutionDependentDiscreteSource3,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
            SolutionDependentDiscreteSource3,
            GetLinearInUValues,
            );
    }

};
void register_SolutionDependentDiscreteSource3_class(py::module &m){
py::class_<SolutionDependentDiscreteSource3 , SolutionDependentDiscreteSource3_Overloads , std::shared_ptr<SolutionDependentDiscreteSource3 >  , DiscreteSource<3>  >(m, "SolutionDependentDiscreteSource3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<SolutionDependentDiscreteSource<3> >(*)()) &SolutionDependentDiscreteSource3::Create, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(SolutionDependentDiscreteSource3::*)()) &SolutionDependentDiscreteSource3::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(SolutionDependentDiscreteSource3::*)()) &SolutionDependentDiscreteSource3::GetLinearInUValues, 
            " "  )
        .def(
            "SetSolution", 
            (void(SolutionDependentDiscreteSource3::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >)) &SolutionDependentDiscreteSource3::SetSolution, 
            " " , py::arg("solution") )
        .def(
            "SetConstantInUSinkRatePerSolutionQuantity", 
            (void(SolutionDependentDiscreteSource3::*)(::QRate)) &SolutionDependentDiscreteSource3::SetConstantInUSinkRatePerSolutionQuantity, 
            " " , py::arg("value") )
        .def(
            "SetLinearInUSinkRatePerSolutionQuantity", 
            (void(SolutionDependentDiscreteSource3::*)(::QRatePerConcentration)) &SolutionDependentDiscreteSource3::SetLinearInUSinkRatePerSolutionQuantity, 
            " " , py::arg("value") )
    ;
}
