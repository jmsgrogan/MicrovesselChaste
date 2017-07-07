#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DiscreteSource.hpp"

#include "DiscreteSource2.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteSource<2 > DiscreteSource2;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1;

class DiscreteSource2_Overloads : public DiscreteSource2{
    public:
    using DiscreteSource2::DiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1,
            DiscreteSource2,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1,
            DiscreteSource2,
            GetLinearInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetNonlinearTermValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1,
            DiscreteSource2,
            GetNonlinearTermValues,
            );
    }
    void UpdateDensityMap() override {
        PYBIND11_OVERLOAD(
            void,
            DiscreteSource2,
            UpdateDensityMap,
            );
    }

};
void register_DiscreteSource2_class(py::module &m){
py::class_<DiscreteSource2 , DiscreteSource2_Overloads   >(m, "DiscreteSource2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteSource<2> >(*)()) &DiscreteSource2::Create, 
            " "  )
        .def(
            "GetDensityMap", 
            (::std::shared_ptr<DensityMap<2> >(DiscreteSource2::*)()) &DiscreteSource2::GetDensityMap, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(DiscreteSource2::*)()) &DiscreteSource2::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(DiscreteSource2::*)()) &DiscreteSource2::GetLinearInUValues, 
            " "  )
        .def(
            "GetNonlinearTermValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(DiscreteSource2::*)()) &DiscreteSource2::GetNonlinearTermValues, 
            " "  )
        .def(
            "SetLabelName", 
            (void(DiscreteSource2::*)(::std::string const &)) &DiscreteSource2::SetLabelName, 
            " " , py::arg("rLabel") )
        .def(
            "GetNumberOfPoints", 
            (unsigned int(DiscreteSource2::*)()) &DiscreteSource2::GetNumberOfPoints, 
            " "  )
        .def(
            "SetPoints", 
            (void(DiscreteSource2::*)(::std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > >)) &DiscreteSource2::SetPoints, 
            " " , py::arg("points") )
        .def(
            "SetConstantInUValue", 
            (void(DiscreteSource2::*)(::QConcentrationFlowRate)) &DiscreteSource2::SetConstantInUValue, 
            " " , py::arg("value") )
        .def(
            "SetLinearInUValue", 
            (void(DiscreteSource2::*)(::QRate)) &DiscreteSource2::SetLinearInUValue, 
            " " , py::arg("value") )
        .def(
            "SetDensityMap", 
            (void(DiscreteSource2::*)(::std::shared_ptr<DensityMap<2> >)) &DiscreteSource2::SetDensityMap, 
            " " , py::arg("pMap") )
        .def(
            "UpdateDensityMap", 
            (void(DiscreteSource2::*)()) &DiscreteSource2::UpdateDensityMap, 
            " "  )
    ;
}
