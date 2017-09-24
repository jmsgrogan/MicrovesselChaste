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

#include "PythonObjectConverters.hpp"
#include "DiscreteContinuumMeshActorGenerator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef DiscreteContinuumMeshActorGenerator<3 > DiscreteContinuumMeshActorGenerator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class DiscreteContinuumMeshActorGenerator3_Overloads : public DiscreteContinuumMeshActorGenerator3{
    public:
    using DiscreteContinuumMeshActorGenerator3::DiscreteContinuumMeshActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            DiscreteContinuumMeshActorGenerator3,
            AddActor,
            pRenderer);
    }

};
void register_DiscreteContinuumMeshActorGenerator3_class(py::module &m){
py::class_<DiscreteContinuumMeshActorGenerator3 , DiscreteContinuumMeshActorGenerator3_Overloads , std::shared_ptr<DiscreteContinuumMeshActorGenerator3 >  , AbstractActorGenerator<3>  >(m, "DiscreteContinuumMeshActorGenerator3")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(DiscreteContinuumMeshActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &DiscreteContinuumMeshActorGenerator3::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetDiscreteContinuumMesh", 
            (void(DiscreteContinuumMeshActorGenerator3::*)(::std::shared_ptr<DiscreteContinuumMesh<3, 3> >)) &DiscreteContinuumMeshActorGenerator3::SetDiscreteContinuumMesh, 
            " " , py::arg("pDiscreteContinuumMesh") )
    ;
}
