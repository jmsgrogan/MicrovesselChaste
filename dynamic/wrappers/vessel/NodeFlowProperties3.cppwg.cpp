#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "NodeFlowProperties.hpp"

#include "NodeFlowProperties3.cppwg.hpp"

namespace py = pybind11;
typedef NodeFlowProperties<3 > NodeFlowProperties3;
;
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class NodeFlowProperties3_Overloads : public NodeFlowProperties3{
    public:
    using NodeFlowProperties3::NodeFlowProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            NodeFlowProperties3,
            GetOutputData,
            );
    }

};
void register_NodeFlowProperties3_class(py::module &m){
py::class_<NodeFlowProperties3 , NodeFlowProperties3_Overloads   >(m, "NodeFlowProperties3")
        .def(py::init< >())
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(NodeFlowProperties3::*)() const ) &NodeFlowProperties3::GetOutputData, 
            " " )
        .def(
            "IsInputNode", 
            (bool(NodeFlowProperties3::*)() const ) &NodeFlowProperties3::IsInputNode, 
            " " )
        .def(
            "IsOutputNode", 
            (bool(NodeFlowProperties3::*)() const ) &NodeFlowProperties3::IsOutputNode, 
            " " )
        .def(
            "SetIsInputNode", 
            (void(NodeFlowProperties3::*)(bool)) &NodeFlowProperties3::SetIsInputNode, 
            " " , py::arg("isInput"))
        .def(
            "SetIsOutputNode", 
            (void(NodeFlowProperties3::*)(bool)) &NodeFlowProperties3::SetIsOutputNode, 
            " " , py::arg("isOutput"))
        .def(
            "SetUseVelocityBoundaryCondition", 
            (void(NodeFlowProperties3::*)(bool)) &NodeFlowProperties3::SetUseVelocityBoundaryCondition, 
            " " , py::arg("useVelocity"))
        .def(
            "UseVelocityBoundaryCondition", 
            (bool(NodeFlowProperties3::*)()) &NodeFlowProperties3::UseVelocityBoundaryCondition, 
            " " )
    ;
}
