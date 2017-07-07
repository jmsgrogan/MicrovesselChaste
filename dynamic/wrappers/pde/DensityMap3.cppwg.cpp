#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DensityMap.hpp"

#include "DensityMap3.cppwg.hpp"

namespace py = pybind11;
typedef DensityMap<3 > DensityMap3;
;

void register_DensityMap3_class(py::module &m){
py::class_<DensityMap3    >(m, "DensityMap3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DensityMap<3> >(*)()) &DensityMap3::Create, 
            " "  )
        .def(
            "GetVesselNetwork", 
            (::std::shared_ptr<VesselNetwork<3> >(DensityMap3::*)()) &DensityMap3::GetVesselNetwork, 
            " "  )
        .def(
            "GetSamplingGrid", 
            (::vtkSmartPointer<vtkUnstructuredGrid>(DensityMap3::*)(::vtkSmartPointer<vtkUnstructuredGrid>)) &DensityMap3::GetSamplingGrid, 
            " " , py::arg("pGrid") )
        .def(
            "GetSamplingGrid", 
            (::vtkSmartPointer<vtkUnstructuredGrid>(DensityMap3::*)(::std::shared_ptr<RegularGrid<3> >)) &DensityMap3::GetSamplingGrid, 
            " " , py::arg("pGrid") )
        .def(
            "GetGridCalculator", 
            (::std::shared_ptr<GridCalculator<3> >(DensityMap3::*)()) &DensityMap3::GetGridCalculator, 
            " "  )
        .def(
            "rGetVesselSurfaceAreaDensity", 
            (::std::vector<double, std::allocator<double> > const &(DensityMap3::*)(bool)) &DensityMap3::rGetVesselSurfaceAreaDensity, 
            " " , py::arg("update") = true )
        .def(
            "rGetVesselLineDensity", 
            (::std::vector<double, std::allocator<double> >(DensityMap3::*)(bool)) &DensityMap3::rGetVesselLineDensity, 
            " " , py::arg("update") = true )
        .def(
            "rGetPerfusedVesselSurfaceAreaDensity", 
            (::std::vector<double, std::allocator<double> > const &(DensityMap3::*)(bool)) &DensityMap3::rGetPerfusedVesselSurfaceAreaDensity, 
            " " , py::arg("update") = true )
        .def(
            "rGetPerfusedVesselLineDensity", 
            (::std::vector<double, std::allocator<double> > const &(DensityMap3::*)(bool)) &DensityMap3::rGetPerfusedVesselLineDensity, 
            " " , py::arg("update") = true )
        .def(
            "rGetVesselTipDensity", 
            (::std::vector<double, std::allocator<double> > const &(DensityMap3::*)(bool)) &DensityMap3::rGetVesselTipDensity, 
            " " , py::arg("update") = true )
        .def(
            "rGetVesselBranchDensity", 
            (::std::vector<double, std::allocator<double> > const &(DensityMap3::*)(bool)) &DensityMap3::rGetVesselBranchDensity, 
            " " , py::arg("update") = true )
        .def(
            "rGetVesselQuantityDensity", 
            (::std::vector<double, std::allocator<double> > const &(DensityMap3::*)(::std::string const &, bool)) &DensityMap3::rGetVesselQuantityDensity, 
            " " , py::arg("rQuantity"), py::arg("update") = true )
        .def(
            "rGetCellDensity", 
            (::std::vector<double, std::allocator<double> > const &(DensityMap3::*)(bool)) &DensityMap3::rGetCellDensity, 
            " " , py::arg("update") = true )
        .def(
            "rGetCellDensity", 
            (::std::vector<double, std::allocator<double> > const &(DensityMap3::*)(::boost::shared_ptr<AbstractCellMutationState>, bool)) &DensityMap3::rGetCellDensity, 
            " " , py::arg("pMutationState"), py::arg("update") = true )
        .def(
            "IsPointInCell", 
            (bool(DensityMap3::*)(::vtkSmartPointer<vtkCellLocator>, ::boost::numeric::ublas::c_vector<double, 3>, unsigned int)) &DensityMap3::IsPointInCell, 
            " " , py::arg("pCellLocator"), py::arg("loc"), py::arg("index") )
        .def(
            "LengthOfLineInCell", 
            (double(DensityMap3::*)(::vtkSmartPointer<vtkUnstructuredGrid>, ::boost::numeric::ublas::c_vector<double, 3>, ::boost::numeric::ublas::c_vector<double, 3>, unsigned int, bool, bool)) &DensityMap3::LengthOfLineInCell, 
            " " , py::arg("pSamplingGrid"), py::arg("loc1"), py::arg("loc2"), py::arg("index"), py::arg("loc1InCell"), py::arg("loc2InCell") )
        .def(
            "SetVesselNetwork", 
            (void(DensityMap3::*)(::std::shared_ptr<VesselNetwork<3> >)) &DensityMap3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetCellPopulation", 
            (void(DensityMap3::*)(::AbstractCellPopulation<3, 3> &, ::QLength, ::QConcentration)) &DensityMap3::SetCellPopulation, 
            " " , py::arg("rCellPopulation"), py::arg("cellPopulationReferenceLength"), py::arg("cellPopulationReferenceConcentration") )
        .def(
            "SetGrid", 
            (void(DensityMap3::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >)) &DensityMap3::SetGrid, 
            " " , py::arg("pGrid") )
        .def(
            "SetGridCalculator", 
            (void(DensityMap3::*)(::std::shared_ptr<GridCalculator<3> >)) &DensityMap3::SetGridCalculator, 
            " " , py::arg("pGridCalculator") )
    ;
}
