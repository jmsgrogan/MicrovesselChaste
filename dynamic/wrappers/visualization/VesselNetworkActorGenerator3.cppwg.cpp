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
#include "VesselNetworkActorGenerator.hpp"

#include "PythonObjectConverters.hpp"
#include "VesselNetworkActorGenerator3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VesselNetworkActorGenerator<3 > VesselNetworkActorGenerator3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class VesselNetworkActorGenerator3_Overloads : public VesselNetworkActorGenerator3{
    public:
    using VesselNetworkActorGenerator3::VesselNetworkActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            VesselNetworkActorGenerator3,
            AddActor,
            pRenderer);
    }

};
void register_VesselNetworkActorGenerator3_class(py::module &m){
py::class_<VesselNetworkActorGenerator3 , VesselNetworkActorGenerator3_Overloads , std::shared_ptr<VesselNetworkActorGenerator3 >  , AbstractActorGenerator<3>  >(m, "VesselNetworkActorGenerator3")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(VesselNetworkActorGenerator3::*)(::vtkSmartPointer<vtkRenderer>)) &VesselNetworkActorGenerator3::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkActorGenerator3::*)(::std::shared_ptr<VesselNetwork<3> >)) &VesselNetworkActorGenerator3::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
    ;
}
