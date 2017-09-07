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

#include "PythonObjectConverters.hpp"
#include "AbstractVesselNetworkComponent3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AbstractVesselNetworkComponent<3 > AbstractVesselNetworkComponent3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef unsigned int unsignedint;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_;
typedef ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > _std_vector_lt_std_basic_string_lt_char_gt__std_allocator_lt_std_basic_string_lt_char_gt__gt__gt_;
typedef ::QLength _QLength;

class AbstractVesselNetworkComponent3_Overloads : public AbstractVesselNetworkComponent3{
    public:
    using AbstractVesselNetworkComponent3::AbstractVesselNetworkComponent;
    unsigned int GetId() const  override {
        PYBIND11_OVERLOAD(
            unsignedint,
            AbstractVesselNetworkComponent3,
            GetId,
            );
    }
    double GetOutputDataValue(::std::string const & rKey) override {
        PYBIND11_OVERLOAD(
            double,
            AbstractVesselNetworkComponent3,
            GetOutputDataValue,
            rKey);
    }
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() override {
        PYBIND11_OVERLOAD_PURE(
            _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_,
            AbstractVesselNetworkComponent3,
            GetOutputData,
            );
    }
    ::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > > GetOutputDataKeys() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_std_basic_string_lt_char_gt__std_allocator_lt_std_basic_string_lt_char_gt__gt__gt_,
            AbstractVesselNetworkComponent3,
            GetOutputDataKeys,
            );
    }
    ::QLength GetRadius() const  override {
        PYBIND11_OVERLOAD(
            _QLength,
            AbstractVesselNetworkComponent3,
            GetRadius,
            );
    }
    void SetId(unsigned int id) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent3,
            SetId,
            id);
    }
    void SetOutputData(::std::string const & rKey, double value) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent3,
            SetOutputData,
            rKey, 
value);
    }
    void SetRadius(::QLength radius) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractVesselNetworkComponent3,
            SetRadius,
            radius);
    }

};
void register_AbstractVesselNetworkComponent3_class(py::module &m){
py::class_<AbstractVesselNetworkComponent3 , AbstractVesselNetworkComponent3_Overloads , std::shared_ptr<AbstractVesselNetworkComponent3 >   >(m, "AbstractVesselNetworkComponent3")
        .def(py::init< >())
        .def(
            "GetId", 
            (unsigned int(AbstractVesselNetworkComponent3::*)() const ) &AbstractVesselNetworkComponent3::GetId, 
            " "  )
        .def(
            "GetOutputDataValue", 
            (double(AbstractVesselNetworkComponent3::*)(::std::string const &)) &AbstractVesselNetworkComponent3::GetOutputDataValue, 
            " " , py::arg("rKey") )
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(AbstractVesselNetworkComponent3::*)()) &AbstractVesselNetworkComponent3::GetOutputData, 
            " "  )
        .def(
            "GetOutputDataKeys", 
            (::std::vector<std::basic_string<char>, std::allocator<std::basic_string<char> > >(AbstractVesselNetworkComponent3::*)()) &AbstractVesselNetworkComponent3::GetOutputDataKeys, 
            " "  )
        .def(
            "GetRadius", 
            (::QLength(AbstractVesselNetworkComponent3::*)() const ) &AbstractVesselNetworkComponent3::GetRadius, 
            " "  )
        .def(
            "SetId", 
            (void(AbstractVesselNetworkComponent3::*)(unsigned int)) &AbstractVesselNetworkComponent3::SetId, 
            " " , py::arg("id") )
        .def(
            "SetOutputData", 
            (void(AbstractVesselNetworkComponent3::*)(::std::string const &, double)) &AbstractVesselNetworkComponent3::SetOutputData, 
            " " , py::arg("rKey"), py::arg("value") )
        .def(
            "SetRadius", 
            (void(AbstractVesselNetworkComponent3::*)(::QLength)) &AbstractVesselNetworkComponent3::SetRadius, 
            " " , py::arg("radius") )
    ;
}
