#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponent.hpp"

#include "AbstractVesselNetworkComponent2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponent<2 > AbstractVesselNetworkComponent2;
;
typedef unsigned int unsignedint;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;
typedef ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > _std_vectorstd_basic_stringchar_std_allocatorstd_basic_stringchar;
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
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            AbstractVesselNetworkComponent2,
            GetOutputData,
            );
    }
    ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > GetOutputDataKeys() override {
        PYBIND11_OVERLOAD(
            _std_vectorstd_basic_stringchar_std_allocatorstd_basic_stringchar,
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
py::class_<AbstractVesselNetworkComponent2 , AbstractVesselNetworkComponent2_Overloads   >(m, "AbstractVesselNetworkComponent2")
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