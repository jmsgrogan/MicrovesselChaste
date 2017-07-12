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

#include "CellBasedDiscreteSource3.cppwg.hpp"

namespace py = pybind11;
typedef CellBasedDiscreteSource<3 > CellBasedDiscreteSource3;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1;

class CellBasedDiscreteSource3_Overloads : public CellBasedDiscreteSource3{
    public:
    using CellBasedDiscreteSource3::CellBasedDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1,
            CellBasedDiscreteSource3,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1,
            CellBasedDiscreteSource3,
            GetLinearInUValues,
            );
    }
    void UpdateDensityMap() override {
        PYBIND11_OVERLOAD(
            void,
            CellBasedDiscreteSource3,
            UpdateDensityMap,
            );
    }

};
void register_CellBasedDiscreteSource3_class(py::module &m){
py::class_<CellBasedDiscreteSource3 , CellBasedDiscreteSource3_Overloads   >(m, "CellBasedDiscreteSource3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CellBasedDiscreteSource<3> >(*)()) &CellBasedDiscreteSource3::Create, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(CellBasedDiscreteSource3::*)()) &CellBasedDiscreteSource3::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(CellBasedDiscreteSource3::*)()) &CellBasedDiscreteSource3::GetLinearInUValues, 
            " "  )
        .def(
            "SetConstantInUConsumptionRatePerCell", 
            (void(CellBasedDiscreteSource3::*)(::QMolarFlowRate)) &CellBasedDiscreteSource3::SetConstantInUConsumptionRatePerCell, 
            " " , py::arg("value") )
        .def(
            "SetLinearInUConsumptionRatePerCell", 
            (void(CellBasedDiscreteSource3::*)(::QRate)) &CellBasedDiscreteSource3::SetLinearInUConsumptionRatePerCell, 
            " " , py::arg("value") )
        .def(
            "UpdateDensityMap", 
            (void(CellBasedDiscreteSource3::*)()) &CellBasedDiscreteSource3::UpdateDensityMap, 
            " "  )
    ;
}
