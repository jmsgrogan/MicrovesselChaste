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
#include "PartActorGenerator.hpp"

#include "PartActorGenerator3.cppwg.hpp"

namespace py = pybind11;
typedef PartActorGenerator<3 > PartActorGenerator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class PartActorGenerator3_Overloads : public PartActorGenerator3{
    public:
    using PartActorGenerator3::PartActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            PartActorGenerator3,
            AddActor,
            pRenderer);
    }

};
void register_PartActorGenerator3_class(py::module &m){
py::class_<PartActorGenerator3 , PartActorGenerator3_Overloads , std::shared_ptr<PartActorGenerator3 >   >(m, "PartActorGenerator3")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(PartActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &PartActorGenerator3::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetPart", 
            (void(PartActorGenerator3::*)(::std::shared_ptr<Part<3> >)) &PartActorGenerator3::SetPart, 
            " " , py::arg("pPart") )
    ;
}
