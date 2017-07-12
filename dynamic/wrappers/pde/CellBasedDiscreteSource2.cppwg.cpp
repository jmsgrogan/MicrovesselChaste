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

#include "CellBasedDiscreteSource2.cppwg.hpp"

namespace py = pybind11;
typedef CellBasedDiscreteSource<2 > CellBasedDiscreteSource2;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1;

class CellBasedDiscreteSource2_Overloads : public CellBasedDiscreteSource2{
    public:
    using CellBasedDiscreteSource2::CellBasedDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1,
            CellBasedDiscreteSource2,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1,
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
py::class_<CellBasedDiscreteSource2 , CellBasedDiscreteSource2_Overloads   >(m, "CellBasedDiscreteSource2")
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
