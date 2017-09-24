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
#include "CellBasedDiscreteSource.hpp"

#include "PythonObjectConverters.hpp"
#include "CellBasedDiscreteSource2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef CellBasedDiscreteSource<2 > CellBasedDiscreteSource2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;

class CellBasedDiscreteSource2_Overloads : public CellBasedDiscreteSource2{
    public:
    using CellBasedDiscreteSource2::CellBasedDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
            CellBasedDiscreteSource2,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_neg1_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
            CellBasedDiscreteSource2,
            GetLinearInUValues,
            );
    }
    void UpdateDensityMap() override {
        PYBIND11_OVERLOAD(
            void,
            CellBasedDiscreteSource2,
            UpdateDensityMap,
            );
    }

};
void register_CellBasedDiscreteSource2_class(py::module &m){
py::class_<CellBasedDiscreteSource2 , CellBasedDiscreteSource2_Overloads , std::shared_ptr<CellBasedDiscreteSource2 >  , DiscreteSource<2>  >(m, "CellBasedDiscreteSource2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CellBasedDiscreteSource<2> >(*)()) &CellBasedDiscreteSource2::Create, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(CellBasedDiscreteSource2::*)()) &CellBasedDiscreteSource2::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(CellBasedDiscreteSource2::*)()) &CellBasedDiscreteSource2::GetLinearInUValues, 
            " "  )
        .def(
            "SetConstantInUConsumptionRatePerCell", 
            (void(CellBasedDiscreteSource2::*)(::QMolarFlowRate)) &CellBasedDiscreteSource2::SetConstantInUConsumptionRatePerCell, 
            " " , py::arg("value") )
        .def(
            "SetLinearInUConsumptionRatePerCell", 
            (void(CellBasedDiscreteSource2::*)(::QRate)) &CellBasedDiscreteSource2::SetLinearInUConsumptionRatePerCell, 
            " " , py::arg("value") )
        .def(
            "UpdateDensityMap", 
            (void(CellBasedDiscreteSource2::*)()) &CellBasedDiscreteSource2::UpdateDensityMap, 
            " "  )
    ;
}
