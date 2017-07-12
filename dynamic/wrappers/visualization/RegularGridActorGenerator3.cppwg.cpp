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
#include "RegularGridActorGenerator.hpp"

#include "RegularGridActorGenerator3.cppwg.hpp"

namespace py = pybind11;
typedef RegularGridActorGenerator<3 > RegularGridActorGenerator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class RegularGridActorGenerator3_Overloads : public RegularGridActorGenerator3{
    public:
    using RegularGridActorGenerator3::RegularGridActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            RegularGridActorGenerator3,
            AddActor,
            pRenderer);
    }

};
void register_RegularGridActorGenerator3_class(py::module &m){
py::class_<RegularGridActorGenerator3 , RegularGridActorGenerator3_Overloads , std::shared_ptr<RegularGridActorGenerator3 >   >(m, "RegularGridActorGenerator3")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(RegularGridActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &RegularGridActorGenerator3::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetRegularGrid", 
            (void(RegularGridActorGenerator3::*)(::std::shared_ptr<RegularGrid<3> >)) &RegularGridActorGenerator3::SetRegularGrid, 
            " " , py::arg("pRegularGrid") )
        .def(
            "SetEdgeOpacity", 
            (void(RegularGridActorGenerator3::*)(double)) &RegularGridActorGenerator3::SetEdgeOpacity, 
            " " , py::arg("opacity") )
    ;
}
