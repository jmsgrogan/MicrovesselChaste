#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponentProperties.hpp"

#include "AbstractVesselNetworkComponentProperties2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractVesselNetworkComponentProperties<2 > AbstractVesselNetworkComponentProperties2;
;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class AbstractVesselNetworkComponentProperties2_Overloads : public AbstractVesselNetworkComponentProperties2{
    public:
    using AbstractVesselNetworkComponentProperties2::AbstractVesselNetworkComponentProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD_PURE(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            AbstractVesselNetworkComponentProperties2,
            GetOutputData,
            );
    }

};
void register_AbstractVesselNetworkComponentProperties2_class(py::module &m){
py::class_<AbstractVesselNetworkComponentProperties2 , AbstractVesselNetworkComponentProperties2_Overloads   >(m, "AbstractVesselNetworkComponentProperties2")
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(AbstractVesselNetworkComponentProperties2::*)() const ) &AbstractVesselNetworkComponentProperties2::GetOutputData, 
            " "  )
    ;
}
