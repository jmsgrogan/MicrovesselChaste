#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "PartActorGenerator.hpp"

#include "PartActorGenerator2.cppwg.hpp"

namespace py = pybind11;
typedef PartActorGenerator<2 > PartActorGenerator2;
;

class PartActorGenerator2_Overloads : public PartActorGenerator2{
    public:
    using PartActorGenerator2::PartActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            PartActorGenerator2,
            AddActor,
            pRenderer);
    }

};
void register_PartActorGenerator2_class(py::module &m){
py::class_<PartActorGenerator2 , PartActorGenerator2_Overloads   >(m, "PartActorGenerator2")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(PartActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &PartActorGenerator2::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetPart", 
            (void(PartActorGenerator2::*)(::std::shared_ptr<Part<2> >)) &PartActorGenerator2::SetPart, 
            " " , py::arg("pPart") )
    ;
}
