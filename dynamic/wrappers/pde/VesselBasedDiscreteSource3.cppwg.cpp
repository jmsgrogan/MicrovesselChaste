#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "VesselBasedDiscreteSource.hpp"

#include "VesselBasedDiscreteSource3.cppwg.hpp"

namespace py = pybind11;
typedef VesselBasedDiscreteSource<3 > VesselBasedDiscreteSource3;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1;

class VesselBasedDiscreteSource3_Overloads : public VesselBasedDiscreteSource3{
    public:
    using VesselBasedDiscreteSource3::VesselBasedDiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1,
            VesselBasedDiscreteSource3,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1,
            VesselBasedDiscreteSource3,
            GetLinearInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetNonlinearTermValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1,
            VesselBasedDiscreteSource3,
            GetNonlinearTermValues,
            );
    }

};
void register_VesselBasedDiscreteSource3_class(py::module &m){
py::class_<VesselBasedDiscreteSource3 , VesselBasedDiscreteSource3_Overloads   >(m, "VesselBasedDiscreteSource3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<VesselBasedDiscreteSource<3> >(*)()) &VesselBasedDiscreteSource3::Create, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(VesselBasedDiscreteSource3::*)()) &VesselBasedDiscreteSource3::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(VesselBasedDiscreteSource3::*)()) &VesselBasedDiscreteSource3::GetLinearInUValues, 
            " "  )
        .def(
            "GetNonlinearTermValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(VesselBasedDiscreteSource3::*)()) &VesselBasedDiscreteSource3::GetNonlinearTermValues, 
            " "  )
        .def(
            "SetVesselPermeability", 
            (void(VesselBasedDiscreteSource3::*)(::QMembranePermeability)) &VesselBasedDiscreteSource3::SetVesselPermeability, 
            " " , py::arg("value") )
        .def(
            "SetReferenceConcentration", 
            (void(VesselBasedDiscreteSource3::*)(::QConcentration)) &VesselBasedDiscreteSource3::SetReferenceConcentration, 
            " " , py::arg("value") )
        .def(
            "SetHalfMaxUptakeConcentration", 
            (void(VesselBasedDiscreteSource3::*)(::QConcentration)) &VesselBasedDiscreteSource3::SetHalfMaxUptakeConcentration, 
            " " , py::arg("value") )
        .def(
            "SetReferenceHaematocrit", 
            (void(VesselBasedDiscreteSource3::*)(::QDimensionless)) &VesselBasedDiscreteSource3::SetReferenceHaematocrit, 
            " " , py::arg("value") )
        .def(
            "SetUptakeRatePerCell", 
            (void(VesselBasedDiscreteSource3::*)(::QMolarFlowRate)) &VesselBasedDiscreteSource3::SetUptakeRatePerCell, 
            " " , py::arg("ratePerCell") )
        .def(
            "SetNumberOfCellsPerLength", 
            (void(VesselBasedDiscreteSource3::*)(::QPerLength)) &VesselBasedDiscreteSource3::SetNumberOfCellsPerLength, 
            " " , py::arg("cellsPerLength") )
    ;
}
