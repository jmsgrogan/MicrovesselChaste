#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "RegularGrid.hpp"

#include "RegularGrid3.cppwg.hpp"

namespace py = pybind11;
typedef RegularGrid<3 > RegularGrid3;
;
typedef ::Vertex<3> _Vertex3;
typedef ::std::vector<double, std::allocator<double> > const & _std_vectordouble_std_allocatordoubleRef;

class RegularGrid3_Overloads : public RegularGrid3{
    public:
    using RegularGrid3::RegularGrid;
    int GetLocalIndex(unsigned int globalIndex) override {
        PYBIND11_OVERLOAD(
            int,
            RegularGrid3,
            GetLocalIndex,
            globalIndex);
    }
    ::Vertex<3> GetGlobalCellLocation(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Vertex3,
            RegularGrid3,
            GetGlobalCellLocation,
            index);
    }
    ::std::vector<double, std::allocator<double> > const & rGetCellVolumes(bool update, bool jiggle) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordoubleRef,
            RegularGrid3,
            rGetCellVolumes,
            update, 
jiggle);
    }
    void SetUpVtkGrid() override {
        PYBIND11_OVERLOAD(
            void,
            RegularGrid3,
            SetUpVtkGrid,
            );
    }
    void SetUpVtkCellLocator() override {
        PYBIND11_OVERLOAD(
            void,
            RegularGrid3,
            SetUpVtkCellLocator,
            );
    }

};
void register_RegularGrid3_class(py::module &m){
py::class_<RegularGrid3 , RegularGrid3_Overloads   >(m, "RegularGrid3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<RegularGrid<3> >(*)()) &RegularGrid3::Create, 
            " "  )
        .def(
            "CalculateNeighbourData", 
            (void(RegularGrid3::*)()) &RegularGrid3::CalculateNeighbourData, 
            " "  )
        .def(
            "CalculateMooreNeighbourData", 
            (void(RegularGrid3::*)()) &RegularGrid3::CalculateMooreNeighbourData, 
            " "  )
        .def(
            "GenerateFromPart", 
            (void(RegularGrid3::*)(::std::shared_ptr<Part<3> >, ::QLength)) &RegularGrid3::GenerateFromPart, 
            " " , py::arg("pPart"), py::arg("gridSize") )
        .def(
            "GetDistributedVectorFactory", 
            (::std::shared_ptr<DistributedVectorFactory>(RegularGrid3::*)()) &RegularGrid3::GetDistributedVectorFactory, 
            " "  )
        .def(
            "GetGlobalGridIndex", 
            (unsigned int(RegularGrid3::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid3::GetGlobalGridIndex, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex") )
        .def(
            "GetGridIndex", 
            (unsigned int(RegularGrid3::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid3::GetGridIndex, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex") )
        .def(
            "GetLocalIndex", 
            (int(RegularGrid3::*)(unsigned int)) &RegularGrid3::GetLocalIndex, 
            " " , py::arg("globalIndex") )
        .def(
            "GetGlobalCellLocation", 
            (::Vertex<3>(RegularGrid3::*)(unsigned int)) &RegularGrid3::GetGlobalCellLocation, 
            " " , py::arg("index") )
        .def(
            "rGetNeighbourData", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > > const &(RegularGrid3::*)()) &RegularGrid3::rGetNeighbourData, 
            " "  )
        .def(
            "rGetMooreNeighbourData", 
            (::std::vector<std::vector<unsigned int, std::allocator<unsigned int> >, std::allocator<std::vector<unsigned int, std::allocator<unsigned int> > > > const &(RegularGrid3::*)()) &RegularGrid3::rGetMooreNeighbourData, 
            " "  )
        .def(
            "GetDimensions", 
            (::boost::numeric::ublas::c_vector<unsigned int, 3>(RegularGrid3::*)()) &RegularGrid3::GetDimensions, 
            " "  )
        .def(
            "GetExtents", 
            (::boost::numeric::ublas::c_vector<unsigned int, 6>(RegularGrid3::*)()) &RegularGrid3::GetExtents, 
            " "  )
        .def(
            "GetPoint", 
            (::Vertex<3>(RegularGrid3::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid3::GetPoint, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex") )
        .def(
            "GetOrigin", 
            (::Vertex<3>(RegularGrid3::*)()) &RegularGrid3::GetOrigin, 
            " "  )
        .def(
            "GetSpacing", 
            (::QLength(RegularGrid3::*)()) &RegularGrid3::GetSpacing, 
            " "  )
        .def(
            "GetPointBoundingBox", 
            (::boost::numeric::ublas::c_vector<double, 6>(RegularGrid3::*)(unsigned int, unsigned int, unsigned int, bool)) &RegularGrid3::GetPointBoundingBox, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex"), py::arg("jiggle") = false )
        .def(
            "GetPointBoundingBox", 
            (::boost::numeric::ublas::c_vector<double, 6>(RegularGrid3::*)(unsigned int, bool)) &RegularGrid3::GetPointBoundingBox, 
            " " , py::arg("gridIndex"), py::arg("jiggle") = false )
        .def(
            "rGetCellVolumes", 
            (::std::vector<double, std::allocator<double> > const &(RegularGrid3::*)(bool, bool)) &RegularGrid3::rGetCellVolumes, 
            " " , py::arg("update") = false, py::arg("jiggle") = false )
        .def(
            "IsOnBoundary", 
            (bool(RegularGrid3::*)(unsigned int)) &RegularGrid3::IsOnBoundary, 
            " " , py::arg("gridIndex") )
        .def(
            "IsOnBoundary", 
            (bool(RegularGrid3::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid3::IsOnBoundary, 
            " " , py::arg("xIndex"), py::arg("yIndex"), py::arg("zIndex") )
        .def(
            "SetDimensions", 
            (void(RegularGrid3::*)(::boost::numeric::ublas::c_vector<unsigned int, 3>)) &RegularGrid3::SetDimensions, 
            " " , py::arg("dimensions") )
        .def(
            "SetDimensions", 
            (void(RegularGrid3::*)(unsigned int, unsigned int, unsigned int)) &RegularGrid3::SetDimensions, 
            " " , py::arg("x"), py::arg("y"), py::arg("z") = 1 )
        .def(
            "SetOrigin", 
            (void(RegularGrid3::*)(::Vertex<3>)) &RegularGrid3::SetOrigin, 
            " " , py::arg("origin") )
        .def(
            "SetSpacing", 
            (void(RegularGrid3::*)(::QLength)) &RegularGrid3::SetSpacing, 
            " " , py::arg("spacing") )
        .def(
            "SetUpVtkGrid", 
            (void(RegularGrid3::*)()) &RegularGrid3::SetUpVtkGrid, 
            " "  )
        .def(
            "SetUpVtkCellLocator", 
            (void(RegularGrid3::*)()) &RegularGrid3::SetUpVtkCellLocator, 
            " "  )
        .def(
            "UpdateExtents", 
            (void(RegularGrid3::*)()) &RegularGrid3::UpdateExtents, 
            " "  )
        .def(
            "Write", 
            (void(RegularGrid3::*)(::std::shared_ptr<OutputFileHandler>)) &RegularGrid3::Write, 
            " " , py::arg("pFileHandler") )
    ;
}
