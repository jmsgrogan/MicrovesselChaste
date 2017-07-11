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

#include "DiscreteSource3.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteSource<3 > DiscreteSource3;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1;

class DiscreteSource3_Overloads : public DiscreteSource3{
    public:
    using DiscreteSource3::DiscreteSource;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConstantInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1,
            DiscreteSource3,
            GetConstantInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > > GetLinearInUValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio0_1_std_ratio-1_1_std_ratio0_1_std_ratio0_1,
            DiscreteSource3,
            GetLinearInUValues,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetNonlinearTermValues() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio-1_1_std_ratio1_1_std_ratio0_1,
            DiscreteSource3,
            GetNonlinearTermValues,
            );
    }
    void UpdateDensityMap() override {
        PYBIND11_OVERLOAD(
            void,
            DiscreteSource3,
            UpdateDensityMap,
            );
    }

};
void register_DiscreteSource3_class(py::module &m){
py::class_<DiscreteSource3 , DiscreteSource3_Overloads   >(m, "DiscreteSource3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteSource<3> >(*)()) &DiscreteSource3::Create, 
            " "  )
        .def(
            "GetDensityMap", 
            (::std::shared_ptr<DensityMap<3> >(DiscreteSource3::*)()) &DiscreteSource3::GetDensityMap, 
            " "  )
        .def(
            "GetConstantInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(DiscreteSource3::*)()) &DiscreteSource3::GetConstantInUValues, 
            " "  )
        .def(
            "GetLinearInUValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<0, 1>, std::ratio<-1, 1>, std::ratio<0, 1>, std::ratio<0, 1> > > >(DiscreteSource3::*)()) &DiscreteSource3::GetLinearInUValues, 
            " "  )
        .def(
            "GetNonlinearTermValues", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<-1, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(DiscreteSource3::*)()) &DiscreteSource3::GetNonlinearTermValues, 
            " "  )
        .def(
            "SetLabelName", 
            (void(DiscreteSource3::*)(::std::string const &)) &DiscreteSource3::SetLabelName, 
            " " , py::arg("rLabel") )
        .def(
            "GetNumberOfPoints", 
            (unsigned int(DiscreteSource3::*)()) &DiscreteSource3::GetNumberOfPoints, 
            " "  )
        .def(
            "SetPoints", 
            (void(DiscreteSource3::*)(::std::vector<Vertex<3>, std::allocator<Vertex<3> > >)) &DiscreteSource3::SetPoints, 
            " " , py::arg("points") )
        .def(
            "SetConstantInUValue", 
            (void(DiscreteSource3::*)(::QConcentrationFlowRate)) &DiscreteSource3::SetConstantInUValue, 
            " " , py::arg("value") )
        .def(
            "SetLinearInUValue", 
            (void(DiscreteSource3::*)(::QRate)) &DiscreteSource3::SetLinearInUValue, 
            " " , py::arg("value") )
        .def(
            "SetDensityMap", 
            (void(DiscreteSource3::*)(::std::shared_ptr<DensityMap<3> >)) &DiscreteSource3::SetDensityMap, 
            " " , py::arg("pMap") )
        .def(
            "UpdateDensityMap", 
            (void(DiscreteSource3::*)()) &DiscreteSource3::UpdateDensityMap, 
            " "  )
    ;
}
