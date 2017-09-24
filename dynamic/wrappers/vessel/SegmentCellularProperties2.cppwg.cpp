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
#include "SegmentCellularProperties.hpp"

#include "PythonObjectConverters.hpp"
#include "SegmentCellularProperties2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef SegmentCellularProperties<2 > SegmentCellularProperties2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_;

class SegmentCellularProperties2_Overloads : public SegmentCellularProperties2{
    public:
    using SegmentCellularProperties2::SegmentCellularProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD(
            _std_map_lt_std_basic_string_lt_char_gt__double_std_less_lt_std_basic_string_lt_char_gt__gt__std_allocator_lt_std_pair_lt_conststd_basic_string_lt_char_gt__double_gt__gt__gt_,
            SegmentCellularProperties2,
            GetOutputData,
            );
    }

};
void register_SegmentCellularProperties2_class(py::module &m){
py::class_<SegmentCellularProperties2 , SegmentCellularProperties2_Overloads , std::shared_ptr<SegmentCellularProperties2 >  , AbstractVesselNetworkComponentCellularProperties<2>  >(m, "SegmentCellularProperties2")
        .def(py::init< >())
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(SegmentCellularProperties2::*)() const ) &SegmentCellularProperties2::GetOutputData, 
            " "  )
    ;
}
