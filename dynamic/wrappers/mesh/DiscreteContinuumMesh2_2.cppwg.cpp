#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "DiscreteContinuumMesh.hpp"

#include "DiscreteContinuumMesh2_2.cppwg.hpp"

namespace py = pybind11;
typedef DiscreteContinuumMesh<2,2 > DiscreteContinuumMesh2_2;
;
typedef ::DimensionalChastePoint<2> _DimensionalChastePoint2;
typedef ::std::vector<double, std::allocator<double> > const & _std_vectordouble_std_allocatordoubleRef;

class DiscreteContinuumMesh2_2_Overloads : public DiscreteContinuumMesh2_2{
    public:
    using DiscreteContinuumMesh2_2::DiscreteContinuumMesh;
    ::DimensionalChastePoint<2> GetGlobalCellLocation(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _DimensionalChastePoint2,
            DiscreteContinuumMesh2_2,
            GetGlobalCellLocation,
            index);
    }
    ::std::vector<double, std::allocator<double> > const & rGetCellVolumes(bool update, bool jiggle) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordoubleRef,
            DiscreteContinuumMesh2_2,
            rGetCellVolumes,
            update, 
jiggle);
    }
    void SetUpVtkGrid() override {
        PYBIND11_OVERLOAD(
            void,
            DiscreteContinuumMesh2_2,
            SetUpVtkGrid,
            );
    }

};
void register_DiscreteContinuumMesh2_2_class(py::module &m){
py::class_<DiscreteContinuumMesh2_2 , DiscreteContinuumMesh2_2_Overloads   >(m, "DiscreteContinuumMesh2_2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumMesh<2, 2> >(*)()) &DiscreteContinuumMesh2_2::Create, 
            " "  )
        .def(
            "GetConnectivity", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(DiscreteContinuumMesh2_2::*)()) &DiscreteContinuumMesh2_2::GetConnectivity, 
            " "  )
        .def(
            "GetGlobalCellLocation", 
            (::DimensionalChastePoint<2>(DiscreteContinuumMesh2_2::*)(unsigned int)) &DiscreteContinuumMesh2_2::GetGlobalCellLocation, 
            " " , py::arg("index") )
        .def(
            "GetPointLocations", 
            (::vtkSmartPointer<vtkPoints>(DiscreteContinuumMesh2_2::*)()) &DiscreteContinuumMesh2_2::GetPointLocations, 
            " "  )
        .def(
            "rGetCellVolumes", 
            (::std::vector<double, std::allocator<double> > const &(DiscreteContinuumMesh2_2::*)(bool, bool)) &DiscreteContinuumMesh2_2::rGetCellVolumes, 
            " " , py::arg("update") = false, py::arg("jiggle") = false )
        .def(
            "GetElementPartitioning", 
            (::std::vector<unsigned int, std::allocator<unsigned int> >(DiscreteContinuumMesh2_2::*)()) &DiscreteContinuumMesh2_2::GetElementPartitioning, 
            " "  )
        .def(
            "SetUpVtkGrid", 
            (void(DiscreteContinuumMesh2_2::*)()) &DiscreteContinuumMesh2_2::SetUpVtkGrid, 
            " "  )
        .def(
            "ImportDiscreteContinuumMeshFromTetgen", 
            (void(DiscreteContinuumMesh2_2::*)(::tetgen::tetgenio &, unsigned int, int *, unsigned int, int *, int *, int *, unsigned int, double *)) &DiscreteContinuumMesh2_2::ImportDiscreteContinuumMeshFromTetgen, 
            " " , py::arg("mesherOutput"), py::arg("numberOfElements"), py::arg("elementList"), py::arg("numberOfFaces"), py::arg("faceList"), py::arg("edgeMarkerList"), py::arg("triFaceMarkerList"), py::arg("numberoftetrahedronattributes"), py::arg("tetrahedronattributelist") )
        .def(
            "ImportDiscreteContinuumMeshFromTri", 
            (void(DiscreteContinuumMesh2_2::*)(::triangulateio &, unsigned int, int *, unsigned int, int *, int *, int *, unsigned int, double *)) &DiscreteContinuumMesh2_2::ImportDiscreteContinuumMeshFromTri, 
            " " , py::arg("mesherOutput"), py::arg("numberOfElements"), py::arg("elementList"), py::arg("numberOfFaces"), py::arg("faceList"), py::arg("edgeMarkerList"), py::arg("triFaceMarkerList"), py::arg("numberoftetrahedronattributes"), py::arg("tetrahedronattributelist") )
    ;
}
