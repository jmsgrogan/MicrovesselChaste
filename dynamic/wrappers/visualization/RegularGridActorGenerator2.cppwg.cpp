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

#include "PythonObjectConverters.hpp"
#include "RegularGridActorGenerator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef RegularGridActorGenerator<2 > RegularGridActorGenerator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class RegularGridActorGenerator2_Overloads : public RegularGridActorGenerator2{
    public:
    using RegularGridActorGenerator2::RegularGridActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            RegularGridActorGenerator2,
            AddActor,
            pRenderer);
    }

};
void register_RegularGridActorGenerator2_class(py::module &m){
py::class_<RegularGridActorGenerator2 , RegularGridActorGenerator2_Overloads , std::shared_ptr<RegularGridActorGenerator2 >  , AbstractActorGenerator<2>  >(m, "RegularGridActorGenerator2")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(RegularGridActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &RegularGridActorGenerator2::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetRegularGrid", 
            (void(RegularGridActorGenerator2::*)(::std::shared_ptr<RegularGrid<2> >)) &RegularGridActorGenerator2::SetRegularGrid, 
            " " , py::arg("pRegularGrid") )
        .def(
            "SetEdgeOpacity", 
            (void(RegularGridActorGenerator2::*)(double)) &RegularGridActorGenerator2::SetEdgeOpacity, 
            " " , py::arg("opacity") )
    ;
}
