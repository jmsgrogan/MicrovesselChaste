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
#include "RegularGrid.hpp"

#include "RegularGrid2.cppwg.hpp"

namespace py = pybind11;
typedef RegularGrid<2 > RegularGrid2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::Vertex<2> _Vertex_lt_2_gt_;
typedef ::std::vector<double, std::allocator<double> > const & _std_vector_lt_double_std_allocator_lt_double_gt__gt_constRef;

class RegularGrid2_Overloads : public RegularGrid2{
    public:
    using RegularGrid2::RegularGrid;
    int GetLocalIndex(unsigned int globalIndex) override {
        PYBIND11_OVERLOAD(
            int,
            RegularGrid2,
            GetLocalIndex,
            globalIndex);
    }
    ::Vertex<2> GetGlobalCellLocation(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex_lt_2_gt_,
            RegularGrid2,
            GetGlobalCellLocation,
            index);
    }
    ::std::vector<double, std::allocator<double> > const & rGetCellVolumes(bool update, bool jiggle) override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_double_std_allocator_lt_double_gt__gt_constRef,
            RegularGrid2,
            rGetCellVolumes,
            update, 
jiggle);
    }
    void SetUpVtkGrid() override {
        PYBIND11_OVERLOAD(
            void,
            RegularGrid2,
            SetUpVtkGrid,
            );
    }
    void SetUpVtkCellLocator() override {
        PYBIND11_OVERLOAD(
            void,
            RegularGrid2,
            SetUpVtkCellLocator,
            );
    }

};
void register_RegularGrid2_class(py::module &m){
py::class_<RegularGrid2 , RegularGrid2_Overloads , std::shared_ptr<RegularGrid2 >  , AbstractDiscreteContinuumGrid<2, 2>  >(m, "RegularGrid2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<RegularGrid<2> >(*)()) &RegularGrid2::Create, 
            " "  )
        .def(
            "CalculateNeighbourData", 
            (void(RegularGrid2::*)()) &RegularGrid2::CalculateNeighbourData, 
            " "  )
        .def(
            "CalculateMooreNeighbourData", 
            (void(RegularGrid2::*)()) &RegularGrid2::CalculateMooreNeighbourData, 
            " "  )
        .def(
            "GenerateFromPart", 
            (void(RegularGrid2::*)(::std::shared_ptr<Part<2> >, ::QLength)) &RegularGrid2::GenerateFromPart, 
            " " , py::arg("pPart"), py::arg("gridSize") )
        .def(
            "GetDistributedVectorFactory", 
            (::std::shared_ptr<DistributedVectorFactory>(RegularGrid2::*)()) &RegularGrid2::GetDistributedVectorFactory, 
            " "  )
        .def(
            "GetGlobalGridIndex", 
            (unsigned int(RegularGrid2::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid2::GetGlobalGridIndex, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex") )
        .def(
            "GetGridIndex", 
            (unsigned int(RegularGrid2::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid2::GetGridIndex, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex") )
        .def(
            "GetLocalIndex", 
            (int(RegularGrid2::*)(unsigned int)) &RegularGrid2::GetLocalIndex, 
            " " , py::arg("globalIndex") )
        .def(
            "GetGlobalCellLocation", 
            (::Vertex<2>(RegularGrid2::*)(unsigned int)) &RegularGrid2::GetGlobalCellLocation, 
            " " , py::arg("index") )
        .def(
            "rGetNeighbourData", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > > const &(RegularGrid2::*)()) &RegularGrid2::rGetNeighbourData, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "rGetMooreNeighbourData", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > > const &(RegularGrid2::*)()) &RegularGrid2::rGetMooreNeighbourData, 
            " "  , py::return_value_policy::reference_internal)
        .def(
            "GetDimensions", 
            (::boost::numeric::ublas::c_vector<unsigned int, 3>(RegularGrid2::*)()) &RegularGrid2::GetDimensions, 
            " "  )
        .def(
            "GetExtents", 
            (::boost::numeric::ublas::c_vector<unsigned int, 6>(RegularGrid2::*)()) &RegularGrid2::GetExtents, 
            " "  )
        .def(
            "GetPoint", 
            (::Vertex<2>(RegularGrid2::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid2::GetPoint, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex") )
        .def(
            "GetOrigin", 
            (::Vertex<2>(RegularGrid2::*)()) &RegularGrid2::GetOrigin, 
            " "  )
        .def(
            "GetSpacing", 
            (::QLength(RegularGrid2::*)()) &RegularGrid2::GetSpacing, 
            " "  )
        .def(
            "GetPointBoundingBox", 
            (::boost::numeric::ublas::c_vector<double, 6>(RegularGrid2::*)(unsigned int, unsigned int, unsigned int, bool)) &RegularGrid2::GetPointBoundingBox, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex"), py::arg("jiggle") = false )
        .def(
            "GetPointBoundingBox", 
            (::boost::numeric::ublas::c_vector<double, 6>(RegularGrid2::*)(unsigned int, bool)) &RegularGrid2::GetPointBoundingBox, 
            " " , py::arg("gridIndex"), py::arg("jiggle") = false )
        .def(
            "rGetCellVolumes", 
            (::std::vector<double, std::allocator<double> > const &(RegularGrid2::*)(bool, bool)) &RegularGrid2::rGetCellVolumes, 
            " " , py::arg("update") = false, py::arg("jiggle") = false , py::return_value_policy::reference_internal)
        .def(
            "IsOnBoundary", 
            (bool(RegularGrid2::*)(unsigned int)) &RegularGrid2::IsOnBoundary, 
            " " , py::arg("gridIndex") )
        .def(
            "IsOnBoundary", 
            (bool(RegularGrid2::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid2::IsOnBoundary, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex") )
        .def(
            "SetDimensions", 
            (void(RegularGrid2::*)(::boost::numeric::ublas::c_vector<unsigned int, 3>)) &RegularGrid2::SetDimensions, 
            " " , py::arg("dimensions") )
        .def(
            "SetDimensions", 
            (void(RegularGrid2::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid2::SetDimensions, 
            " " , py::arg("x"), py::arg("y"), py::arg("z") = 1 )
        .def(
            "SetOrigin", 
            (void(RegularGrid2::*)(::Vertex<2>)) &RegularGrid2::SetOrigin, 
            " " , py::arg("origin") )
        .def(
            "SetSpacing", 
            (void(RegularGrid2::*)(::QLength)) &RegularGrid2::SetSpacing, 
            " " , py::arg("spacing") )
        .def(
            "SetUpVtkGrid", 
            (void(RegularGrid2::*)()) &RegularGrid2::SetUpVtkGrid, 
            " "  )
        .def(
            "SetUpVtkCellLocator", 
            (void(RegularGrid2::*)()) &RegularGrid2::SetUpVtkCellLocator, 
            " "  )
        .def(
            "UpdateExtents", 
            (void(RegularGrid2::*)()) &RegularGrid2::UpdateExtents, 
            " "  )
        .def(
            "Write", 
            (void(RegularGrid2::*)(::std::shared_ptr<OutputFileHandler>)) &RegularGrid2::Write, 
            " " , py::arg("pFileHandler") )
    ;
}
