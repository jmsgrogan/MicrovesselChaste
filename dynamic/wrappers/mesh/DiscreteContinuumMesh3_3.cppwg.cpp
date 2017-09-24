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
#include "DiscreteContinuumMesh.hpp"

#include "PythonObjectConverters.hpp"
#include "DiscreteContinuumMesh3_3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef DiscreteContinuumMesh<3,3 > DiscreteContinuumMesh3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::Vertex<3> _Vertex_lt_3_gt_;
typedef ::std::vector<double, std::allocator<double> > const & _std_vector_lt_double_std_allocator_lt_double_gt__gt_constRef;
typedef ::QLength _QLength;

class DiscreteContinuumMesh3_3_Overloads : public DiscreteContinuumMesh3_3{
    public:
    using DiscreteContinuumMesh3_3::DiscreteContinuumMesh;
    ::Vertex<3> GetGlobalCellLocation(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex_lt_3_gt_,
            DiscreteContinuumMesh3_3,
            GetGlobalCellLocation,
            index);
    }
    ::std::vector<double, std::allocator<double> > const & rGetCellVolumes(bool update, bool jiggle) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_double_std_allocator_lt_double_gt__gt_constRef,
            DiscreteContinuumMesh3_3,
            rGetCellVolumes,
            update, 
jiggle);
    }
    ::QLength GetSpacing() override {
        PYBIND11_OVERLOAD(
            _QLength,
            DiscreteContinuumMesh3_3,
            GetSpacing,
            );
    }
    void SetUpVtkGrid() override {
        PYBIND11_OVERLOAD(
            void,
            DiscreteContinuumMesh3_3,
            SetUpVtkGrid,
            );
    }

};
void register_DiscreteContinuumMesh3_3_class(py::module &m){
py::class_<DiscreteContinuumMesh3_3 , DiscreteContinuumMesh3_3_Overloads , std::shared_ptr<DiscreteContinuumMesh3_3 >  , AbstractDiscreteContinuumGrid<3, 3>  >(m, "DiscreteContinuumMesh3_3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<DiscreteContinuumMesh<3, 3> >(*)()) &DiscreteContinuumMesh3_3::Create, 
            " "  )
        .def(
            "GetConnectivity", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > >(DiscreteContinuumMesh3_3::*)()) &DiscreteContinuumMesh3_3::GetConnectivity, 
            " "  )
        .def(
            "GetGlobalCellLocation", 
            (::Vertex<3>(DiscreteContinuumMesh3_3::*)(unsigned int)) &DiscreteContinuumMesh3_3::GetGlobalCellLocation, 
            " " , py::arg("index") )
        .def(
            "GetPointLocations", 
            (::vtkSmartPointer<vtkPoints>(DiscreteContinuumMesh3_3::*)()) &DiscreteContinuumMesh3_3::GetPointLocations, 
            " "  )
        .def(
            "rGetCellVolumes", 
            (::std::vector<double, std::allocator<double> > const &(DiscreteContinuumMesh3_3::*)(bool, bool)) &DiscreteContinuumMesh3_3::rGetCellVolumes, 
            " " , py::arg("update") = false, py::arg("jiggle") = false , py::return_value_policy::reference_internal)
        .def(
            "GetElementPartitioning", 
            (::std::vector<unsigned int, std::allocator<unsigned int> >(DiscreteContinuumMesh3_3::*)()) &DiscreteContinuumMesh3_3::GetElementPartitioning, 
            " "  )
        .def(
            "GetSpacing", 
            (::QLength(DiscreteContinuumMesh3_3::*)()) &DiscreteContinuumMesh3_3::GetSpacing, 
            " "  )
        .def(
            "SetUpVtkGrid", 
            (void(DiscreteContinuumMesh3_3::*)()) &DiscreteContinuumMesh3_3::SetUpVtkGrid, 
            " "  )
        .def(
            "ImportDiscreteContinuumMeshFromTetgen", 
            (void(DiscreteContinuumMesh3_3::*)(::tetgen::tetgenio &, unsigned int, int *, unsigned int, int *, int *, int *, unsigned int, double *)) &DiscreteContinuumMesh3_3::ImportDiscreteContinuumMeshFromTetgen, 
            " " , py::arg("mesherOutput"), py::arg("numberOfElements"), py::arg("elementList"), py::arg("numberOfFaces"), py::arg("faceList"), py::arg("edgeMarkerList"), py::arg("triFaceMarkerList"), py::arg("numberoftetrahedronattributes"), py::arg("tetrahedronattributelist") )
        .def(
            "ImportDiscreteContinuumMeshFromTri", 
            (void(DiscreteContinuumMesh3_3::*)(::triangulateio &, unsigned int, int *, unsigned int, int *, int *, int *, unsigned int, double *)) &DiscreteContinuumMesh3_3::ImportDiscreteContinuumMeshFromTri, 
            " " , py::arg("mesherOutput"), py::arg("numberOfElements"), py::arg("elementList"), py::arg("numberOfFaces"), py::arg("faceList"), py::arg("edgeMarkerList"), py::arg("triFaceMarkerList"), py::arg("numberoftetrahedronattributes"), py::arg("tetrahedronattributelist") )
    ;
}
