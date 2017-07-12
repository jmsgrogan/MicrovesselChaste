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
#include "NodeFlowProperties.hpp"

#include "NodeFlowProperties2.cppwg.hpp"

namespace py = pybind11;
typedef NodeFlowProperties<2 > NodeFlowProperties2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double;

class NodeFlowProperties2_Overloads : public NodeFlowProperties2{
    public:
    using NodeFlowProperties2::NodeFlowProperties;
    ::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > > GetOutputData() const  override {
        PYBIND11_OVERLOAD(
            _std_mapstd_basic_stringchar_double_std_lessstd_basic_stringchar_std_allocatorstd_pairstd_basic_stringchar_double,
            NodeFlowProperties2,
            GetOutputData,
            );
    }

};
void register_NodeFlowProperties2_class(py::module &m){
py::class_<NodeFlowProperties2 , NodeFlowProperties2_Overloads , std::shared_ptr<NodeFlowProperties2 >   >(m, "NodeFlowProperties2")
        .def(py::init< >())
        .def(
            "GetOutputData", 
            (::std::map<std::basic_string<char>, double, std::less<std::basic_string<char> >, std::allocator<std::pair<const std::basic_string<char>, double> > >(NodeFlowProperties2::*)() const ) &NodeFlowProperties2::GetOutputData, 
            " "  )
        .def(
            "IsInputNode", 
            (bool(NodeFlowProperties2::*)() const ) &NodeFlowProperties2::IsInputNode, 
            " "  )
        .def(
            "IsOutputNode", 
            (bool(NodeFlowProperties2::*)() const ) &NodeFlowProperties2::IsOutputNode, 
            " "  )
        .def(
            "SetIsInputNode", 
            (void(NodeFlowProperties2::*)(bool)) &NodeFlowProperties2::SetIsInputNode, 
            " " , py::arg("isInput") )
        .def(
            "SetIsOutputNode", 
            (void(NodeFlowProperties2::*)(bool)) &NodeFlowProperties2::SetIsOutputNode, 
            " " , py::arg("isOutput") )
        .def(
            "SetUseVelocityBoundaryCondition", 
            (void(NodeFlowProperties2::*)(bool)) &NodeFlowProperties2::SetUseVelocityBoundaryCondition, 
            " " , py::arg("useVelocity") )
        .def(
            "UseVelocityBoundaryCondition", 
            (bool(NodeFlowProperties2::*)()) &NodeFlowProperties2::UseVelocityBoundaryCondition, 
            " "  )
    ;
}
