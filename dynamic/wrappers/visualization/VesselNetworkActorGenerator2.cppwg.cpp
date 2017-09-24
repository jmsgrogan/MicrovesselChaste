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
#include "VesselNetworkActorGenerator2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef VesselNetworkActorGenerator<2 > VesselNetworkActorGenerator2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class VesselNetworkActorGenerator2_Overloads : public VesselNetworkActorGenerator2{
    public:
    using VesselNetworkActorGenerator2::VesselNetworkActorGenerator;
    void AddActor(::vtkSmartPointer<vtkRenderer> pRenderer) override {
        PYBIND11_OVERLOAD(
            void,
            VesselNetworkActorGenerator2,
            AddActor,
            pRenderer);
    }

};
void register_VesselNetworkActorGenerator2_class(py::module &m){
py::class_<VesselNetworkActorGenerator2 , VesselNetworkActorGenerator2_Overloads , std::shared_ptr<VesselNetworkActorGenerator2 >  , AbstractActorGenerator<2>  >(m, "VesselNetworkActorGenerator2")
        .def(py::init< >())
        .def(
            "AddActor", 
            (void(VesselNetworkActorGenerator2::*)(::vtkSmartPointer<vtkRenderer>)) &VesselNetworkActorGenerator2::AddActor, 
            " " , py::arg("pRenderer") )
        .def(
            "SetVesselNetwork", 
            (void(VesselNetworkActorGenerator2::*)(::std::shared_ptr<VesselNetwork<2> >)) &VesselNetworkActorGenerator2::SetVesselNetwork, 
            " " , py::arg("pVesselNetwork") )
    ;
}
