#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "CellStateDependentDiscreteSource.hpp"

#include "CellStateDependentDiscreteSource3.cppwg.hpp"

namespace py = pybind11;
typedef CellStateDependentDiscreteSource<3 > CellStateDependentDiscreteSource3;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1;

class CellStateDependentDiscreteSource3_Overloads : public CellStateDependentDiscreteSource3{
    public:
    using CellStateDependentDiscreteSource3::CellStateDependentDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1,
            CellStateDependentDiscreteSource3,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1,
            CellStateDependentDiscreteSource3,
            GetLinearInUValues,
            );
    }

};
void register_CellStateDependentDiscreteSource3_class(py::module &m){
py::class_<CellStateDependentDiscreteSource3 , CellStateDependentDiscreteSource3_Overloads   >(m, "CellStateDependentDiscreteSource3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CellStateDependentDiscreteSource<3> >(*)()) &CellStateDependentDiscreteSource3::Create, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(CellStateDependentDiscreteSource3::*)()) &CellStateDependentDiscreteSource3::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(CellStateDependentDiscreteSource3::*)()) &CellStateDependentDiscreteSource3::GetLinearInUValues, 
            " "  )
        .def(
            "SetStateRateMap", 
            (void(CellStateDependentDiscreteSource3::*)(::std::map<unsigned int, RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > >)) &CellStateDependentDiscreteSource3::SetStateRateMap, 
            " " , py::arg("stateRateMap") )
        .def(
            "SetStateRateThresholdMap", 
            (void(CellStateDependentDiscreteSource3::*)(::std::map<unsigned int, RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > >)) &CellStateDependentDiscreteSource3::SetStateRateThresholdMap, 
            " " , py::arg("stateThresholdMap") )
    ;
}