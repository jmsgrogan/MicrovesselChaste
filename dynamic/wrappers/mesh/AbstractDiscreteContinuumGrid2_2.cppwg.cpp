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

#include "AbstractDiscreteContinuumGrid2_2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumGrid<2,2 > AbstractDiscreteContinuumGrid2_2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::vtkSmartPointer<vtkPolyData> _vtkSmartPointer_lt_vtkPolyData_gt_;
typedef unsigned int unsignedint;
typedef ::Vertex<2> _Vertex_lt_2_gt_;
typedef ::Vertex<2> _Vertex_lt_2_gt_;
typedef ::Vertex<2> _Vertex_lt_2_gt_;
typedef ::Vertex<2> _Vertex_lt_2_gt_;
typedef ::std::vector<double, std::allocator<double> > const & _std_vector_lt_double_std_allocator_lt_double_gt__gt_constRef;
typedef ::vtkSmartPointer<vtkDataSet> _vtkSmartPointer_lt_vtkDataSet_gt_;
typedef ::vtkSmartPointer<vtkDataSet> _vtkSmartPointer_lt_vtkDataSet_gt_;
typedef ::vtkSmartPointer<vtkPoints> _vtkSmartPointer_lt_vtkPoints_gt_;
typedef ::vtkSmartPointer<vtkCellLocator> _vtkSmartPointer_lt_vtkCellLocator_gt_;

class AbstractDiscreteContinuumGrid2_2_Overloads : public AbstractDiscreteContinuumGrid2_2{
    public:
    using AbstractDiscreteContinuumGrid2_2::AbstractDiscreteContinuumGrid;
    void AddPointData(::std::vector<double, std::allocator<double> > const & rPointValues, ::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            AddPointData,
            rPointValues, 
rName);
    }
    void AddCellData(::std::vector<double, std::allocator<double> > const & rCellValues, ::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            AddCellData,
            rCellValues, 
rName);
    }
    ::vtkSmartPointer<vtkPolyData> GetBoundingGeometry() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkPolyData_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetBoundingGeometry,
            );
    }
    unsigned int GetGlobalIndex(unsigned int localIndex) override {
        PYBIND11_OVERLOAD(
            unsignedint,
            AbstractDiscreteContinuumGrid2_2,
            GetGlobalIndex,
            localIndex);
    }
    int GetLocalIndex(unsigned int globalIndex) override {
        PYBIND11_OVERLOAD(
            int,
            AbstractDiscreteContinuumGrid2_2,
            GetLocalIndex,
            globalIndex);
    }
    ::Vertex<2> GetGlobalPoint(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex_lt_2_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetGlobalPoint,
            index);
    }
    ::Vertex<2> GetPoint(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex_lt_2_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetPoint,
            index);
    }
    ::Vertex<2> GetGlobalCellLocation(unsigned int index) override {
        PYBIND11_OVERLOAD_PURE(
            _Vertex_lt_2_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetGlobalCellLocation,
            index);
    }
    ::Vertex<2> GetCellLocation(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex_lt_2_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetCellLocation,
            index);
    }
    ::std::vector<double, std::allocator<double> > const & rGetCellVolumes(bool update, bool jiggle) override {
        PYBIND11_OVERLOAD_PURE(
            _std_vector_lt_double_std_allocator_lt_double_gt__gt_constRef,
            AbstractDiscreteContinuumGrid2_2,
            rGetCellVolumes,
            update, 
jiggle);
    }
    ::vtkSmartPointer<vtkDataSet> GetGlobalVtkGrid() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkDataSet_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetGlobalVtkGrid,
            );
    }
    ::vtkSmartPointer<vtkDataSet> GetVtkGrid() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkDataSet_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetVtkGrid,
            );
    }
    void GatherPointData(::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            GatherPointData,
            rName);
    }
    void GatherAllPointData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            GatherAllPointData,
            );
    }
    void AllGatherPointData(::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            AllGatherPointData,
            rName);
    }
    void AllGatherAllPointData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            AllGatherAllPointData,
            );
    }
    void GatherCellData(::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            GatherCellData,
            rName);
    }
    void GatherAllCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            GatherAllCellData,
            );
    }
    void AllGatherCellData(::std::string const & rName) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            AllGatherCellData,
            rName);
    }
    void AllGatherAllCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            AllGatherAllCellData,
            );
    }
    ::vtkSmartPointer<vtkPoints> GetPoints() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkPoints_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetPoints,
            );
    }
    ::vtkSmartPointer<vtkCellLocator> GetVtkCellLocator() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointer_lt_vtkCellLocator_gt_,
            AbstractDiscreteContinuumGrid2_2,
            GetVtkCellLocator,
            );
    }
    void SetUpVtkGrid() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumGrid2_2,
            SetUpVtkGrid,
            );
    }
    void SetUpVtkCellLocator() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumGrid2_2,
            SetUpVtkCellLocator,
            );
    }

};
void register_AbstractDiscreteContinuumGrid2_2_class(py::module &m){
py::class_<AbstractDiscreteContinuumGrid2_2 , AbstractDiscreteContinuumGrid2_2_Overloads , std::shared_ptr<AbstractDiscreteContinuumGrid2_2 >   >(m, "AbstractDiscreteContinuumGrid2_2")
        .def(py::init< >())
        .def(
            "AddPointAttributes", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::vector<unsigned int, std::allocator<unsigned int> > const &, ::std::string const &)) &AbstractDiscreteContinuumGrid2_2::AddPointAttributes, 
            " " , py::arg("rAttributes"), py::arg("rName") = "Default Attribute" )
        .def(
            "AddCellAttributes", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::vector<unsigned int, std::allocator<unsigned int> > const &, ::std::string const &)) &AbstractDiscreteContinuumGrid2_2::AddCellAttributes, 
            " " , py::arg("rAttributes"), py::arg("rName") = "Default Attribute" )
        .def(
            "AddPointData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::vector<double, std::allocator<double> > const &, ::std::string const &)) &AbstractDiscreteContinuumGrid2_2::AddPointData, 
            " " , py::arg("rPointValues"), py::arg("rName") = "Default Data" )
        .def(
            "AddCellData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::vector<double, std::allocator<double> > const &, ::std::string const &)) &AbstractDiscreteContinuumGrid2_2::AddCellData, 
            " " , py::arg("rCellValues"), py::arg("rName") = "Default Data" )
        .def(
            "GetAttributesKeys", 
            (::std::map<unsigned int, std::basic_string<char>, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, std::basic_string<char> > > >(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetAttributesKeys, 
            " "  )
        .def(
            "GetBoundingGeometry", 
            (::vtkSmartPointer<vtkPolyData>(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetBoundingGeometry, 
            " "  )
        .def(
            "GetGlobalIndex", 
            (unsigned int(AbstractDiscreteContinuumGrid2_2::*)(unsigned int)) &AbstractDiscreteContinuumGrid2_2::GetGlobalIndex, 
            " " , py::arg("localIndex") )
        .def(
            "GetLocalIndex", 
            (int(AbstractDiscreteContinuumGrid2_2::*)(unsigned int)) &AbstractDiscreteContinuumGrid2_2::GetLocalIndex, 
            " " , py::arg("globalIndex") )
        .def(
            "GetPointData", 
            (::vtkSmartPointer<vtkPointData>(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetPointData, 
            " "  )
        .def(
            "GetCellData", 
            (::vtkSmartPointer<vtkCellData>(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetCellData, 
            " "  )
        .def(
            "CalculateDistanceMap", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumGrid2_2::*)(::std::shared_ptr<Part<2> >)) &AbstractDiscreteContinuumGrid2_2::CalculateDistanceMap, 
            " " , py::arg("pSamplePart") )
        .def(
            "GetGlobalPoint", 
            (::Vertex<2>(AbstractDiscreteContinuumGrid2_2::*)(unsigned int)) &AbstractDiscreteContinuumGrid2_2::GetGlobalPoint, 
            " " , py::arg("index") )
        .def(
            "GetPoint", 
            (::Vertex<2>(AbstractDiscreteContinuumGrid2_2::*)(unsigned int)) &AbstractDiscreteContinuumGrid2_2::GetPoint, 
            " " , py::arg("index") )
        .def(
            "GetGlobalCellLocation", 
            (::Vertex<2>(AbstractDiscreteContinuumGrid2_2::*)(unsigned int)) &AbstractDiscreteContinuumGrid2_2::GetGlobalCellLocation, 
            " " , py::arg("index") )
        .def(
            "GetCellLocation", 
            (::Vertex<2>(AbstractDiscreteContinuumGrid2_2::*)(unsigned int)) &AbstractDiscreteContinuumGrid2_2::GetCellLocation, 
            " " , py::arg("index") )
        .def(
            "rGetCellVolumes", 
            (::std::vector<double, std::allocator<double> > const &(AbstractDiscreteContinuumGrid2_2::*)(bool, bool)) &AbstractDiscreteContinuumGrid2_2::rGetCellVolumes, 
            " " , py::arg("update") = false, py::arg("jiggle") = false , py::return_value_policy::reference_internal)
        .def(
            "GetGlobalVtkGrid", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetGlobalVtkGrid, 
            " "  )
        .def(
            "GetVtkGrid", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetVtkGrid, 
            " "  )
        .def(
            "GatherPointData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::string const &)) &AbstractDiscreteContinuumGrid2_2::GatherPointData, 
            " " , py::arg("rName") )
        .def(
            "GatherAllPointData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GatherAllPointData, 
            " "  )
        .def(
            "AllGatherPointData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::string const &)) &AbstractDiscreteContinuumGrid2_2::AllGatherPointData, 
            " " , py::arg("rName") )
        .def(
            "AllGatherAllPointData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::AllGatherAllPointData, 
            " "  )
        .def(
            "GatherCellData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::string const &)) &AbstractDiscreteContinuumGrid2_2::GatherCellData, 
            " " , py::arg("rName") )
        .def(
            "GatherAllCellData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GatherAllCellData, 
            " "  )
        .def(
            "AllGatherCellData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::string const &)) &AbstractDiscreteContinuumGrid2_2::AllGatherCellData, 
            " " , py::arg("rName") )
        .def(
            "AllGatherAllCellData", 
            (void(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::AllGatherAllCellData, 
            " "  )
        .def(
            "GetNearestCellIndex", 
            (unsigned int(AbstractDiscreteContinuumGrid2_2::*)(::Vertex<2> const &)) &AbstractDiscreteContinuumGrid2_2::GetNearestCellIndex, 
            " " , py::arg("rLocation") )
        .def(
            "GetNumberOfPoints", 
            (unsigned int(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetNumberOfPoints, 
            " "  )
        .def(
            "GetNumberOfCells", 
            (unsigned int(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetNumberOfCells, 
            " "  )
        .def(
            "GetPoints", 
            (::vtkSmartPointer<vtkPoints>(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetPoints, 
            " "  )
        .def(
            "GetCellLocations", 
            (::vtkSmartPointer<vtkPoints>(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetCellLocations, 
            " "  )
        .def(
            "GetReferenceLengthScale", 
            (::QLength(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetReferenceLengthScale, 
            " "  )
        .def(
            "GetVtkCellLocator", 
            (::vtkSmartPointer<vtkCellLocator>(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::GetVtkCellLocator, 
            " "  )
        .def(
            "SetAttributesKeys", 
            (void(AbstractDiscreteContinuumGrid2_2::*)(::std::map<unsigned int, std::basic_string<char>, std::less<unsigned int>, std::allocator<std::pair<const unsigned int, std::basic_string<char> > > >)) &AbstractDiscreteContinuumGrid2_2::SetAttributesKeys, 
            " " , py::arg("attributeKeys") )
        .def(
            "SetUpVtkGrid", 
            (void(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::SetUpVtkGrid, 
            " "  )
        .def(
            "SetUpVtkCellLocator", 
            (void(AbstractDiscreteContinuumGrid2_2::*)()) &AbstractDiscreteContinuumGrid2_2::SetUpVtkCellLocator, 
            " "  )
    ;
}
