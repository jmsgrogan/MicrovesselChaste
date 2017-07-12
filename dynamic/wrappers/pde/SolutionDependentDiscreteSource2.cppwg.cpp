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

#include "SolutionDependentDiscreteSource2.cppwg.hpp"

namespace py = pybind11;
typedef SolutionDependentDiscreteSource<2 > SolutionDependentDiscreteSource2;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1;

class SolutionDependentDiscreteSource2_Overloads : public SolutionDependentDiscreteSource2{
    public:
    using SolutionDependentDiscreteSource2::SolutionDependentDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1,
            SolutionDependentDiscreteSource2,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1,
            SolutionDependentDiscreteSource2,
            GetLinearInUValues,
            );
    }

};
void register_SolutionDependentDiscreteSource2_class(py::module &m){
py::class_<SolutionDependentDiscreteSource2 , SolutionDependentDiscreteSource2_Overloads   >(m, "SolutionDependentDiscreteSource2")
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
