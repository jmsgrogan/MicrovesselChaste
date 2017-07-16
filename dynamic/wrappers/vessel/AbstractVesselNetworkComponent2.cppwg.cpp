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
#include "AbstractVesselNetworkComponent.hpp"

#include "AbstractVesselNetworkComponent2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponent<2 > AbstractVesselNetworkComponent2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef unsigned int unsignedint;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_;
typedef ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > _std_vector_lt_std_basic_string_lt_char_gt__std_allocator_lt_std_basic_string_lt_char_gt__gt__gt_;
typedef ::QLength _QLength;

class AbstractVesselNetworkComponent2_Overloads : public AbstractVesselNetworkComponent2{
    public:
    using AbstractVesselNetworkComponent2::AbstractVesselNetworkComponent;
    unsigned int GetId() const  override {
        PYBIND11_OVERLOAD(
            unsignedint,
            AbstractVesselNetworkComponent2,
            GetId,
            );
    }
    double GetOutputDataValue(::std::string const & rKey) override {
        PYBIND11_OVERLOAD(
            double,
            AbstractVesselNetworkComponent2,
            GetOutputDataValue,
            rKey);
    }
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD_PURE(
            _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_,
            AbstractVesselNetworkComponent2,
            GetOutputData,
            );
    }
    ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > GetOutputDataKeys() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_std_basic_string_lt_char_gt__std_allocator_lt_std_basic_string_lt_char_gt__gt__gt_,
            AbstractVesselNetworkComponent2,
            GetOutputDataKeys,
            );
    }
    ::QLength GetRadius() const  override {
        PYBIND11_OVERLOAD(
            _QLength,
            AbstractVesselNetworkComponent2,
            GetRadius,
            );
    }
    void SetId(unsigned int id) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent2,
            SetId,
            id);
    }
    void SetOutputData(::std::string const & rKey, double value) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent2,
            SetOutputData,
            rKey, 
value);
    }
    void SetRadius(::QLength radius) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent2,
            SetRadius,
            radius);
    }

};
void register_AbstractVesselNetworkComponent2_class(py::module &m){
py::class_<AbstractVesselNetworkComponent2 , AbstractVesselNetworkComponent2_Overloads , std::shared_ptr<AbstractVesselNetworkComponent2 >   >(m, "AbstractVesselNetworkComponent2")
        .def(py::init< >())
        .def(
            "GetId", 
            (unsigned int(AbstractVesselNetworkComponent2::*)() const ) &AbstractVesselNetworkComponent2::GetId, 
            " "  )
        .def(
            "GetOutputDataValue", 
            (double(AbstractVesselNetworkComponent2::*)(::std::string const &)) &AbstractVesselNetworkComponent2::GetOutputDataValue, 
            " " , py::arg("rKey") )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(AbstractVesselNetworkComponent2::*)()) &AbstractVesselNetworkComponent2::GetOutputData, 
            " "  )
        .def(
            "GetOutputDataKeys", 
            (::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >(AbstractVesselNetworkComponent2::*)()) &AbstractVesselNetworkComponent2::GetOutputDataKeys, 
            " "  )
        .def(
            "GetRadius", 
            (::QLength(AbstractVesselNetworkComponent2::*)() const ) &AbstractVesselNetworkComponent2::GetRadius, 
            " "  )
        .def(
            "SetId", 
            (void(AbstractVesselNetworkComponent2::*)(unsigned int)) &AbstractVesselNetworkComponent2::SetId, 
            " " , py::arg("id") )
        .def(
            "SetOutputData", 
            (void(AbstractVesselNetworkComponent2::*)(::std::string const &, double)) &AbstractVesselNetworkComponent2::SetOutputData, 
            " " , py::arg("rKey"), py::arg("value") )
        .def(
            "SetRadius", 
            (void(AbstractVesselNetworkComponent2::*)(::QLength)) &AbstractVesselNetworkComponent2::SetRadius, 
            " " , py::arg("radius") )
    ;
}
