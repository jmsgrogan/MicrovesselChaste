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
#include "VesselBasedDiscreteSource.hpp"

#include "VesselBasedDiscreteSource2.cppwg.hpp"

namespace py = pybind11;
typedef VesselBasedDiscreteSource<2 > VesselBasedDiscreteSource2;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1;

class VesselBasedDiscreteSource2_Overloads : public VesselBasedDiscreteSource2{
    public:
    using VesselBasedDiscreteSource2::VesselBasedDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1,
            VesselBasedDiscreteSource2,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_rationeg1_1_std_ratio0_1_std_ratio0_1,
            VesselBasedDiscreteSource2,
            GetLinearInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetNonlinearTermValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_rationeg1_1_std_ratio1_1_std_ratio0_1,
            VesselBasedDiscreteSource2,
            GetNonlinearTermValues,
            );
    }

};
void register_VesselBasedDiscreteSource2_class(py::module &m){
py::class_<VesselBasedDiscreteSource2 , VesselBasedDiscreteSource2_Overloads   >(m, "VesselBasedDiscreteSource2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselBasedDiscreteSource<2> >(*)()) &VesselBasedDiscreteSource2::Create, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(VesselBasedDiscreteSource2::*)()) &VesselBasedDiscreteSource2::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(VesselBasedDiscreteSource2::*)()) &VesselBasedDiscreteSource2::GetLinearInUValues, 
            " "  )
        .def(
            "GetNonlinearTermValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(VesselBasedDiscreteSource2::*)()) &VesselBasedDiscreteSource2::GetNonlinearTermValues, 
            " "  )
        .def(
            "SetVesselPermeability", 
            (void(VesselBasedDiscreteSource2::*)(::QMembranePermeability)) &VesselBasedDiscreteSource2::SetVesselPermeability, 
            " " , py::arg("value") )
        .def(
            "SetReferenceConcentration", 
            (void(VesselBasedDiscreteSource2::*)(::QConcentration)) &VesselBasedDiscreteSource2::SetReferenceConcentration, 
            " " , py::arg("value") )
        .def(
            "SetHalfMaxUptakeConcentration", 
            (void(VesselBasedDiscreteSource2::*)(::QConcentration)) &VesselBasedDiscreteSource2::SetHalfMaxUptakeConcentration, 
            " " , py::arg("value") )
        .def(
            "SetReferenceHaematocrit", 
            (void(VesselBasedDiscreteSource2::*)(::QDimensionless)) &VesselBasedDiscreteSource2::SetReferenceHaematocrit, 
            " " , py::arg("value") )
        .def(
            "SetUptakeRatePerCell", 
            (void(VesselBasedDiscreteSource2::*)(::QMolarFlowRate)) &VesselBasedDiscreteSource2::SetUptakeRatePerCell, 
            " " , py::arg("ratePerCell") )
        .def(
            "SetNumberOfCellsPerLength", 
            (void(VesselBasedDiscreteSource2::*)(::QPerLength)) &VesselBasedDiscreteSource2::SetNumberOfCellsPerLength, 
            " " , py::arg("cellsPerLength") )
    ;
}
