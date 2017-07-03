#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DiscreteContinuumMeshActorGenerator.hpp"

#include "DiscreteContinuumMeshActorGenerator2.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumMeshActorGenerator<2 > DiscreteContinuumMeshActorGenerator2;
;

class DiscreteContinuumMeshActorGenerator2_Overloads : public DiscreteContinuumMeshActorGenerator2{
    public:
    using DiscreteContinuumMeshActorGenerator2::DiscreteContinuumMeshActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            DiscreteContinuumMeshActorGenerator2,
            AddActor,
            pRenderer);
    }

};
void register_DiscreteContinuumMeshActorGenerator2_class(py::module &m){
py::class_<DiscreteContinuumMeshActorGenerator2 , DiscreteContinuumMeshActorGenerator2_Overloads   >(m, "DiscreteContinuumMeshActorGenerator2")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(DiscreteContinuumMeshActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &DiscreteContinuumMeshActorGenerator2::AddActor, 
            " " , py::arg("pRenderer"))
        .def(
            "SetDiscreteContinuumMesh", 
            (void(DiscreteContinuumMeshActorGenerator2::*)(::std::shared_ptr<DiscreteContinuumMesh<2, 2> >)) &DiscreteContinuumMeshActorGenerator2::SetDiscreteContinuumMesh, 
            " " , py::arg("pDiscreteContinuumMesh"))
    ;
}
