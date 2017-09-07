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
#include "AbstractDiscreteContinuumGrid.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractDiscreteContinuumGrid3_3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AbstractDiscreteContinuumGrid<3,3 > AbstractDiscreteContinuumGrid3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::vtkSmartPointer<vtkPolyData> _vtkSmartPointer_lt_vtkPolyData_gt_;
typedef unsigned int unsignedint;
typedef ::Vertex<3> _Vertex_lt_3_gt_;
typedef ::Vertex<3> _Vertex_lt_3_gt_;
typedef ::Vertex<3> _Vertex_lt_3_gt_;
typedef ::Vertex<3> _Vertex_lt_3_gt_;
typedef ::QLength _QLength;
typedef ::std::vector<double, std::allocator<double> > const & _std_vector_lt_double_std_allocator_lt_double_gt__gt_constRef;
typedef ::vtkSmartPointer<vtkDataSet> _vtkSmartPointer_lt_vtkDataSet_gt_;
typedef ::vtkSmartPointer<vtkDataSet> _vtkSmartPointer_lt_vtkDataSet_gt_;
typedef ::vtkSmartPointer<vtkPoints> _vtkSmartPointer_lt_vtkPoints_gt_;
typedef ::vtkSmartPointer<vtkCellLocator> _vtkSmartPointer_lt_vtkCellLocator_gt_;

class AbstractDiscreteContinuumGrid3_3_Overloads : public AbstractDiscreteContinuumGrid3_3{
    public:
    using AbstractDiscreteContinuumGrid3_3::AbstractDiscreteContinuumGrid;
    void AddPointData(::std::vector<double, std::allocator<double> > const & rPointValues, ::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            AddPointData,
            rPointValues, 
rName);
    }
    void AddCellData(::std::vector<double, std::allocator<double> > const & rCellValues, ::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            AddCellData,
            rCellValues, 
rName);
    }
    ::vtkSmartPointer<vtkPolyData> GetBoundingGeometry() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkPolyData_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetBoundingGeometry,
            );
    }
    unsigned int GetGlobalIndex(unsigned int localIndex) override {
        PYBIND11_OVERLOAD(
            unsignedint,
            AbstractDiscreteContinuumGrid3_3,
            GetGlobalIndex,
            localIndex);
    }
    int GetLocalIndex(unsigned int globalIndex) override {
        PYBIND11_OVERLOAD(
            int,
            AbstractDiscreteContinuumGrid3_3,
            GetLocalIndex,
            globalIndex);
    }
    ::Vertex<3> GetGlobalPoint(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex_lt_3_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetGlobalPoint,
            index);
    }
    ::Vertex<3> GetPoint(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex_lt_3_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetPoint,
            index);
    }
    ::Vertex<3> GetGlobalCellLocation(unsigned int index) override {
        PYBIND11_OVERLOAD_PURE(
            _Vertex_lt_3_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetGlobalCellLocation,
            index);
    }
    ::Vertex<3> GetCellLocation(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex_lt_3_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetCellLocation,
            index);
    }
    ::QLength GetSpacing() override {
        PYBIND11_OVERLOAD_PURE(
            _QLength,
            AbstractDiscreteContinuumGrid3_3,
            GetSpacing,
            );
    }
    ::std::vector<double, std::allocator<double> > const & rGetCellVolumes(bool update, bool jiggle) override {
        PYBIND11_OVERLOAD_PURE(
            _std_vector_lt_double_std_allocator_lt_double_gt__gt_constRef,
            AbstractDiscreteContinuumGrid3_3,
            rGetCellVolumes,
            update, 
jiggle);
    }
    ::vtkSmartPointer<vtkDataSet> GetGlobalVtkGrid() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkDataSet_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetGlobalVtkGrid,
            );
    }
    ::vtkSmartPointer<vtkDataSet> GetVtkGrid() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkDataSet_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetVtkGrid,
            );
    }
    void GatherPointData(::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            GatherPointData,
            rName);
    }
    void GatherAllPointData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            GatherAllPointData,
            );
    }
    void AllGatherPointData(::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            AllGatherPointData,
            rName);
    }
    void AllGatherAllPointData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            AllGatherAllPointData,
            );
    }
    void GatherCellData(::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            GatherCellData,
            rName);
    }
    void GatherAllCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            GatherAllCellData,
            );
    }
    void AllGatherCellData(::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            AllGatherCellData,
            rName);
    }
    void AllGatherAllCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            AllGatherAllCellData,
            );
    }
    ::vtkSmartPointer<vtkPoints> GetPoints() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkPoints_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetPoints,
            );
    }
    ::vtkSmartPointer<vtkCellLocator> GetVtkCellLocator() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkCellLocator_gt_,
            AbstractDiscreteContinuumGrid3_3,
            GetVtkCellLocator,
            );
    }
    void SetUpVtkGrid() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumGrid3_3,
            SetUpVtkGrid,
            );
    }
    void SetUpVtkCellLocator() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid3_3,
            SetUpVtkCellLocator,
            );
    }

};
void register_AbstractDiscreteContinuumGrid3_3_class(py::module &m){
py::class_<AbstractDiscreteContinuumGrid3_3 , AbstractDiscreteContinuumGrid3_3_Overloads , std::shared_ptr<AbstractDiscreteContinuumGrid3_3 >   >(m, "AbstractDiscreteContinuumGrid3_3")
        .def(py::init< >())
        .def(
            "AddPointAttributes", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::vector<unsigned int, std::allocator<unsigned int> > const &, ::std::string const &)) &AbstractDiscreteContinuumGrid3_3::AddPointAttributes, 
            " " , py::arg("rAttributes"), py::arg("rName") = "Default Attribute" )
        .def(
            "AddCellAttributes", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::vector<unsigned int, std::allocator<unsigned int> > const &, ::std::string const &)) &AbstractDiscreteContinuumGrid3_3::AddCellAttributes, 
            " " , py::arg("rAttributes"), py::arg("rName") = "Default Attribute" )
        .def(
            "AddPointData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::vector<double, std::allocator<double> > const &, ::std::string const &)) &AbstractDiscreteContinuumGrid3_3::AddPointData, 
            " " , py::arg("rPointValues"), py::arg("rName") = "Default Data" )
        .def(
            "AddCellData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::vector<double, std::allocator<double> > const &, ::std::string const &)) &AbstractDiscreteContinuumGrid3_3::AddCellData, 
            " " , py::arg("rCellValues"), py::arg("rName") = "Default Data" )
        .def(
            "GetAttributesKeys", 
            (::std::map<unsigned int, std::basic_string<char>, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, std::basic_string<char> > > >(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetAttributesKeys, 
            " "  )
        .def(
            "GetBoundingGeometry", 
            (::vtkSmartPointer<vtkPolyData>(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetBoundingGeometry, 
            " "  )
        .def(
            "GetGlobalIndex", 
            (unsigned int(AbstractDiscreteContinuumGrid3_3::*)(unsigned int)) &AbstractDiscreteContinuumGrid3_3::GetGlobalIndex, 
            " " , py::arg("localIndex") )
        .def(
            "GetLocalIndex", 
            (int(AbstractDiscreteContinuumGrid3_3::*)(unsigned int)) &AbstractDiscreteContinuumGrid3_3::GetLocalIndex, 
            " " , py::arg("globalIndex") )
        .def(
            "GetPointData", 
            (::vtkSmartPointer<vtkPointData>(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetPointData, 
            " "  )
        .def(
            "GetCellData", 
            (::vtkSmartPointer<vtkCellData>(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetCellData, 
            " "  )
        .def(
            "CalculateDistanceMap", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumGrid3_3::*)(::std::shared_ptr<Part<3> >)) &AbstractDiscreteContinuumGrid3_3::CalculateDistanceMap, 
            " " , py::arg("pSamplePart") )
        .def(
            "GetGlobalPoint", 
            (::Vertex<3>(AbstractDiscreteContinuumGrid3_3::*)(unsigned int)) &AbstractDiscreteContinuumGrid3_3::GetGlobalPoint, 
            " " , py::arg("index") )
        .def(
            "GetPoint", 
            (::Vertex<3>(AbstractDiscreteContinuumGrid3_3::*)(unsigned int)) &AbstractDiscreteContinuumGrid3_3::GetPoint, 
            " " , py::arg("index") )
        .def(
            "GetGlobalCellLocation", 
            (::Vertex<3>(AbstractDiscreteContinuumGrid3_3::*)(unsigned int)) &AbstractDiscreteContinuumGrid3_3::GetGlobalCellLocation, 
            " " , py::arg("index") )
        .def(
            "GetCellLocation", 
            (::Vertex<3>(AbstractDiscreteContinuumGrid3_3::*)(unsigned int)) &AbstractDiscreteContinuumGrid3_3::GetCellLocation, 
            " " , py::arg("index") )
        .def(
            "GetSpacing", 
            (::QLength(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetSpacing, 
            " "  )
        .def(
            "rGetCellVolumes", 
            (::std::vector<double, std::allocator<double> > const &(AbstractDiscreteContinuumGrid3_3::*)(bool, bool)) &AbstractDiscreteContinuumGrid3_3::rGetCellVolumes, 
            " " , py::arg("update") = false, py::arg("jiggle") = false , py::return_value_policy::reference_internal)
        .def(
            "GetGlobalVtkGrid", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetGlobalVtkGrid, 
            " "  )
        .def(
            "GetVtkGrid", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetVtkGrid, 
            " "  )
        .def(
            "GatherPointData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::string const &)) &AbstractDiscreteContinuumGrid3_3::GatherPointData, 
            " " , py::arg("rName") )
        .def(
            "GatherAllPointData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GatherAllPointData, 
            " "  )
        .def(
            "AllGatherPointData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::string const &)) &AbstractDiscreteContinuumGrid3_3::AllGatherPointData, 
            " " , py::arg("rName") )
        .def(
            "AllGatherAllPointData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::AllGatherAllPointData, 
            " "  )
        .def(
            "GatherCellData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::string const &)) &AbstractDiscreteContinuumGrid3_3::GatherCellData, 
            " " , py::arg("rName") )
        .def(
            "GatherAllCellData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GatherAllCellData, 
            " "  )
        .def(
            "AllGatherCellData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::string const &)) &AbstractDiscreteContinuumGrid3_3::AllGatherCellData, 
            " " , py::arg("rName") )
        .def(
            "AllGatherAllCellData", 
            (void(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::AllGatherAllCellData, 
            " "  )
        .def(
            "GetNearestCellIndex", 
            (unsigned int(AbstractDiscreteContinuumGrid3_3::*)(::Vertex<3> const &)) &AbstractDiscreteContinuumGrid3_3::GetNearestCellIndex, 
            " " , py::arg("rLocation") )
        .def(
            "GetNumberOfPoints", 
            (unsigned int(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetNumberOfPoints, 
            " "  )
        .def(
            "GetNumberOfCells", 
            (unsigned int(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetNumberOfCells, 
            " "  )
        .def(
            "GetPoints", 
            (::vtkSmartPointer<vtkPoints>(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetPoints, 
            " "  )
        .def(
            "GetCellLocations", 
            (::vtkSmartPointer<vtkPoints>(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetCellLocations, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetVtkCellLocator", 
            (::vtkSmartPointer<vtkCellLocator>(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::GetVtkCellLocator, 
            " "  )
        .def(
            "SetAttributesKeys", 
            (void(AbstractDiscreteContinuumGrid3_3::*)(::std::map<unsigned int, std::basic_string<char>, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, std::basic_string<char> > > >)) &AbstractDiscreteContinuumGrid3_3::SetAttributesKeys, 
            " " , py::arg("attributeKeys") )
        .def(
            "SetUpVtkGrid", 
            (void(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::SetUpVtkGrid, 
            " "  )
        .def(
            "SetUpVtkCellLocator", 
            (void(AbstractDiscreteContinuumGrid3_3::*)()) &AbstractDiscreteContinuumGrid3_3::SetUpVtkCellLocator, 
            " "  )
    ;
}
