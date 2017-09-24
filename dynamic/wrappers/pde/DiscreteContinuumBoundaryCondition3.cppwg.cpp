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
#include "DiscreteContinuumBoundaryCondition.hpp"

#include "PythonObjectConverters.hpp"
#include "DiscreteContinuumBoundaryCondition3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef DiscreteContinuumBoundaryCondition<3 > DiscreteContinuumBoundaryCondition3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_DiscreteContinuumBoundaryCondition3_class(py::module &m){
py::class_<DiscreteContinuumBoundaryCondition3  , std::shared_ptr<DiscreteContinuumBoundaryCondition3 >   >(m, "DiscreteContinuumBoundaryCondition3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumBoundaryCondition<3> >(*)()) &DiscreteContinuumBoundaryCondition3::Create, 
            " "  )
        .def(
            "GetType", 
            (::BoundaryConditionType::Value(DiscreteContinuumBoundaryCondition3::*)()) &DiscreteContinuumBoundaryCondition3::GetType, 
            " "  )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::std::shared_ptr<Part<3> >)) &DiscreteContinuumBoundaryCondition3::SetDomain, 
            " " , py::arg("pDomain") )
        .def(
            "SetIsRobin", 
            (void(DiscreteContinuumBoundaryCondition3::*)(bool)) &DiscreteContinuumBoundaryCondition3::SetIsRobin, 
            " " , py::arg("isRobin") )
        .def(
            "SetIsNeumann", 
            (void(DiscreteContinuumBoundaryCondition3::*)(bool)) &DiscreteContinuumBoundaryCondition3::SetIsNeumann, 
            " " , py::arg("isNeumann") )
        .def(
            "SetLabel", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::std::string const &)) &DiscreteContinuumBoundaryCondition3::SetLabel, 
            " " , py::arg("label") )
        .def(
            "SetPoints", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::std::vector<Vertex<3>, std::allocator<Vertex<3> > >)) &DiscreteContinuumBoundaryCondition3::SetPoints, 
            " " , py::arg("points") )
        .def(
            "SetPoints", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::vtkSmartPointer<vtkPoints>)) &DiscreteContinuumBoundaryCondition3::SetPoints, 
            " " , py::arg("pPoints") )
        .def(
            "SetGridCalculator", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::std::shared_ptr<GridCalculator<3> >)) &DiscreteContinuumBoundaryCondition3::SetGridCalculator, 
            " " , py::arg("pGridCalculator") )
        .def(
            "SetSource", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::BoundaryConditionSource::Value)) &DiscreteContinuumBoundaryCondition3::SetSource, 
            " " , py::arg("boundarySource") )
        .def(
            "SetType", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::BoundaryConditionType::Value)) &DiscreteContinuumBoundaryCondition3::SetType, 
            " " , py::arg("boundaryType") )
        .def(
            "SetNetwork", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::std::shared_ptr<VesselNetwork<3> >)) &DiscreteContinuumBoundaryCondition3::SetNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetValue", 
            (void(DiscreteContinuumBoundaryCondition3::*)(::QConcentration)) &DiscreteContinuumBoundaryCondition3::SetValue, 
            " " , py::arg("value") )
    ;
}
