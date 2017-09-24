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
#include "AbstractVesselNetworkComponentProperties.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractVesselNetworkComponentProperties3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractVesselNetworkComponentProperties<3 > AbstractVesselNetworkComponentProperties3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_;

class AbstractVesselNetworkComponentProperties3_Overloads : public AbstractVesselNetworkComponentProperties3{
    public:
    using AbstractVesselNetworkComponentProperties3::AbstractVesselNetworkComponentProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD_PURE(
            _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_,
            AbstractVesselNetworkComponentProperties3,
            GetOutputData,
            );
    }

};
void register_AbstractVesselNetworkComponentProperties3_class(py::module &m){
py::class_<AbstractVesselNetworkComponentProperties3 , AbstractVesselNetworkComponentProperties3_Overloads , std::shared_ptr<AbstractVesselNetworkComponentProperties3 >   >(m, "AbstractVesselNetworkComponentProperties3")
        .def(py::init< >())
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(AbstractVesselNetworkComponentProperties3::*)() const ) &AbstractVesselNetworkComponentProperties3::GetOutputData, 
            " "  )
    ;
}
