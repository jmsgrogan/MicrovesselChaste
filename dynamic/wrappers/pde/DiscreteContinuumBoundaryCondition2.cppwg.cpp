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
#include "DiscreteContinuumBoundaryCondition2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef DiscreteContinuumBoundaryCondition<2 > DiscreteContinuumBoundaryCondition2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

void register_DiscreteContinuumBoundaryCondition2_class(py::module &m){
py::class_<DiscreteContinuumBoundaryCondition2  , std::shared_ptr<DiscreteContinuumBoundaryCondition2 >   >(m, "DiscreteContinuumBoundaryCondition2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumBoundaryCondition<2> >(*)()) &DiscreteContinuumBoundaryCondition2::Create, 
            " "  )
        .def(
            "GetType", 
            (::BoundaryConditionType::Value(DiscreteContinuumBoundaryCondition2::*)()) &DiscreteContinuumBoundaryCondition2::GetType, 
            " "  )
        .def(
            "SetDomain", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::shared_ptr<Part<2> >)) &DiscreteContinuumBoundaryCondition2::SetDomain, 
            " " , py::arg("pDomain") )
        .def(
            "SetIsRobin", 
            (void(DiscreteContinuumBoundaryCondition2::*)(bool)) &DiscreteContinuumBoundaryCondition2::SetIsRobin, 
            " " , py::arg("isRobin") )
        .def(
            "SetIsNeumann", 
            (void(DiscreteContinuumBoundaryCondition2::*)(bool)) &DiscreteContinuumBoundaryCondition2::SetIsNeumann, 
            " " , py::arg("isNeumann") )
        .def(
            "SetLabel", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::string const &)) &DiscreteContinuumBoundaryCondition2::SetLabel, 
            " " , py::arg("label") )
        .def(
            "SetPoints", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::vector<Vertex<2>, std::allocator<Vertex<2> > >)) &DiscreteContinuumBoundaryCondition2::SetPoints, 
            " " , py::arg("points") )
        .def(
            "SetPoints", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::vtkSmartPointer<vtkPoints>)) &DiscreteContinuumBoundaryCondition2::SetPoints, 
            " " , py::arg("pPoints") )
        .def(
            "SetGridCalculator", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::shared_ptr<GridCalculator<2> >)) &DiscreteContinuumBoundaryCondition2::SetGridCalculator, 
            " " , py::arg("pGridCalculator") )
        .def(
            "SetSource", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::BoundaryConditionSource::Value)) &DiscreteContinuumBoundaryCondition2::SetSource, 
            " " , py::arg("boundarySource") )
        .def(
            "SetType", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::BoundaryConditionType::Value)) &DiscreteContinuumBoundaryCondition2::SetType, 
            " " , py::arg("boundaryType") )
        .def(
            "SetNetwork", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::std::shared_ptr<VesselNetwork<2> >)) &DiscreteContinuumBoundaryCondition2::SetNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetValue", 
            (void(DiscreteContinuumBoundaryCondition2::*)(::QConcentration)) &DiscreteContinuumBoundaryCondition2::SetValue, 
            " " , py::arg("value") )
    ;
}
