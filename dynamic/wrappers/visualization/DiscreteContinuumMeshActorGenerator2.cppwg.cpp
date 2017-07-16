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
#include "DiscreteContinuumMeshActorGenerator.hpp"

#include "DiscreteContinuumMeshActorGenerator2.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumMeshActorGenerator<2 > DiscreteContinuumMeshActorGenerator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

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
py::class_<DiscreteContinuumMeshActorGenerator2 , DiscreteContinuumMeshActorGenerator2_Overloads , std::shared_ptr<DiscreteContinuumMeshActorGenerator2 >  , AbstractActorGenerator<2>  >(m, "DiscreteContinuumMeshActorGenerator2")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(DiscreteContinuumMeshActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &DiscreteContinuumMeshActorGenerator2::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetDiscreteContinuumMesh", 
            (void(DiscreteContinuumMeshActorGenerator2::*)(::std::shared_ptr<DiscreteContinuumMesh<2, 2> >)) &DiscreteContinuumMeshActorGenerator2::SetDiscreteContinuumMesh, 
            " " , py::arg("pDiscreteContinuumMesh") )
    ;
}
