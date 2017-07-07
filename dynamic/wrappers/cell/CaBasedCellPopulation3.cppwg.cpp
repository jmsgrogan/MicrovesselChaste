#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "CaBasedCellPopulation.hpp"

#include "CaBasedCellPopulation3.cppwg.hpp"

namespace py = pybind11;
typedef CaBasedCellPopulation<3 > CaBasedCellPopulation3;
;
typedef ::TetrahedralMesh<3, 3> * _TetrahedralMesh3_3Ptr;
typedef ::Node<3> * _Node3Ptr;
typedef unsigned int unsignedint;
typedef ::std::set<unsigned int, std::less<unsigned int>, std::allocator<unsigned int> > _std_setunsignedint_std_lessunsignedint_std_allocatorunsignedint;
typedef ::boost::numeric::ublas::c_vector<double, 3> _boost_numeric_ublas_c_vectordouble_3;
typedef ::CellPtr _CellPtr;
typedef unsigned int unsignedint;
typedef ::std::vector<boost::shared_ptr<AbstractUpdateRule<3> >, std::allocator<boost::shared_ptr<AbstractUpdateRule<3> > > > const _std_vectorboost_shared_ptrAbstractUpdateRule3_std_allocatorboost_shared_ptrAbstractUpdateRule3;

class CaBasedCellPopulation3_Overloads : public CaBasedCellPopulation3{
    public:
    using CaBasedCellPopulation3::CaBasedCellPopulation;
    bool IsSiteAvailable(unsigned int index, ::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            bool,
            CaBasedCellPopulation3,
            IsSiteAvailable,
            index, 
pCell);
    }
    ::TetrahedralMesh<3, 3> * GetTetrahedralMeshForPdeModifier() override {
        PYBIND11_OVERLOAD(
            _TetrahedralMesh3_3Ptr,
            CaBasedCellPopulation3,
            GetTetrahedralMeshForPdeModifier,
            );
    }
    ::Node<3> * GetNode(unsigned int index) override {
        PYBIND11_OVERLOAD(
            _Node3Ptr,
            CaBasedCellPopulation3,
            GetNode,
            index);
    }
    unsigned int GetNumNodes() override {
        PYBIND11_OVERLOAD(
            unsignedint,
            CaBasedCellPopulation3,
            GetNumNodes,
            );
    }
    ::std::set<unsigned int, std::less<unsigned int>, std::allocator<unsigned int> > GetNeighbouringLocationIndices(::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            _std_setunsignedint_std_lessunsignedint_std_allocatorunsignedint,
            CaBasedCellPopulation3,
            GetNeighbouringLocationIndices,
            pCell);
    }
    ::boost::numeric::ublas::c_vector<double, 3> GetLocationOfCellCentre(::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            _boost_numeric_ublas_c_vectordouble_3,
            CaBasedCellPopulation3,
            GetLocationOfCellCentre,
            pCell);
    }
    void AddCellUsingLocationIndex(unsigned int index, ::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            AddCellUsingLocationIndex,
            index, 
pCell);
    }
    void RemoveCellUsingLocationIndex(unsigned int index, ::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            RemoveCellUsingLocationIndex,
            index, 
pCell);
    }
    ::CellPtr AddCell(::CellPtr pNewCell, ::CellPtr pParentCell) override {
        PYBIND11_OVERLOAD(
            _CellPtr,
            CaBasedCellPopulation3,
            AddCell,
            pNewCell, 
pParentCell);
    }
    double EvaluateDivisionPropensity(unsigned int currentNodeIndex, unsigned int targetNodeIndex, ::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            double,
            CaBasedCellPopulation3,
            EvaluateDivisionPropensity,
            currentNodeIndex, 
targetNodeIndex, 
pCell);
    }
    unsigned int RemoveDeadCells() override {
        PYBIND11_OVERLOAD(
            unsignedint,
            CaBasedCellPopulation3,
            RemoveDeadCells,
            );
    }
    void OpenWritersFiles(::OutputFileHandler & rOutputFileHandler) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            OpenWritersFiles,
            rOutputFileHandler);
    }
    void UpdateCellLocations(double dt) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            UpdateCellLocations,
            dt);
    }
    bool IsCellAssociatedWithADeletedLocation(::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            bool,
            CaBasedCellPopulation3,
            IsCellAssociatedWithADeletedLocation,
            pCell);
    }
    void Update(bool hasHadBirthsOrDeaths) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            Update,
            hasHadBirthsOrDeaths);
    }
    void AcceptPopulationWriter(::boost::shared_ptr<AbstractCellPopulationWriter<3, 3> > pPopulationWriter) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            AcceptPopulationWriter,
            pPopulationWriter);
    }
    void AcceptPopulationCountWriter(::boost::shared_ptr<AbstractCellPopulationCountWriter<3, 3> > pPopulationCountWriter) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            AcceptPopulationCountWriter,
            pPopulationCountWriter);
    }
    void AcceptCellWriter(::boost::shared_ptr<AbstractCellWriter<3, 3> > pCellWriter, ::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            AcceptCellWriter,
            pCellWriter, 
pCell);
    }
    double GetVolumeOfCell(::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            double,
            CaBasedCellPopulation3,
            GetVolumeOfCell,
            pCell);
    }
    double GetWidth(unsigned int const & rDimension) override {
        PYBIND11_OVERLOAD(
            double,
            CaBasedCellPopulation3,
            GetWidth,
            rDimension);
    }
    void RemoveAllUpdateRules() override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            RemoveAllUpdateRules,
            );
    }
    void OutputCellPopulationParameters(::out_stream & rParamsFile) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            OutputCellPopulationParameters,
            rParamsFile);
    }
    bool IsRoomToDivide(::CellPtr pCell) override {
        PYBIND11_OVERLOAD(
            bool,
            CaBasedCellPopulation3,
            IsRoomToDivide,
            pCell);
    }
    void AddUpdateRule(::boost::shared_ptr<AbstractUpdateRule<3> > pUpdateRule) override {
        PYBIND11_OVERLOAD(
            void,
            CaBasedCellPopulation3,
            AddUpdateRule,
            pUpdateRule);
    }
    ::std::vector<boost::shared_ptr<AbstractUpdateRule<3> >, std::allocator<boost::shared_ptr<AbstractUpdateRule<3> > > > const GetUpdateRuleCollection() const  override {
        PYBIND11_OVERLOAD(
            _std_vectorboost_shared_ptrAbstractUpdateRule3_std_allocatorboost_shared_ptrAbstractUpdateRule3,
            CaBasedCellPopulation3,
            GetUpdateRuleCollection,
            );
    }
    double GetCellDataItemAtPdeNode(unsigned int pdeNodeIndex, ::std::string & rVariableName, bool dirichletBoundaryConditionApplies, double dirichletBoundaryValue) override {
        PYBIND11_OVERLOAD(
            double,
            CaBasedCellPopulation3,
            GetCellDataItemAtPdeNode,
            pdeNodeIndex, 
rVariableName, 
dirichletBoundaryConditionApplies, 
dirichletBoundaryValue);
    }
    bool IsPdeNodeAssociatedWithNonApoptoticCell(unsigned int pdeNodeIndex) override {
        PYBIND11_OVERLOAD(
            bool,
            CaBasedCellPopulation3,
            IsPdeNodeAssociatedWithNonApoptoticCell,
            pdeNodeIndex);
    }

};
void register_CaBasedCellPopulation3_class(py::module &m){
py::class_<CaBasedCellPopulation3 , CaBasedCellPopulation3_Overloads   >(m, "CaBasedCellPopulation3")
        .def(py::init<::PottsMesh<3> &, ::std::vector<boost::shared_ptr<Cell>, std::allocator<boost::shared_ptr<Cell> > > &, ::std::vector<unsigned int, std::allocator<unsigned int> > const, unsigned int, bool, bool >(), py::arg("rMesh"), py::arg("rCells"), py::arg("locationIndices"), py::arg("latticeCarryingCapacity") = 1U, py::arg("deleteMesh") = false, py::arg("validate") = false)
        .def(py::init<::PottsMesh<3> & >(), py::arg("rMesh"))
        .def(
            "rGetAvailableSpaces", 
            (::std::vector<unsigned int, std::allocator<unsigned int> > &(CaBasedCellPopulation3::*)()) &CaBasedCellPopulation3::rGetAvailableSpaces, 
            " "  )
        .def(
            "IsSiteAvailable", 
            (bool(CaBasedCellPopulation3::*)(unsigned int, ::CellPtr)) &CaBasedCellPopulation3::IsSiteAvailable, 
            " " , py::arg("index"), py::arg("pCell") )
        .def(
            "rGetMesh", 
            (::PottsMesh<3> &(CaBasedCellPopulation3::*)()) &CaBasedCellPopulation3::rGetMesh, 
            " "  )
        .def(
            "rGetMesh", 
            (::PottsMesh<3> const &(CaBasedCellPopulation3::*)() const ) &CaBasedCellPopulation3::rGetMesh, 
            " "  )
        .def(
            "GetTetrahedralMeshForPdeModifier", 
            (::TetrahedralMesh<3, 3> *(CaBasedCellPopulation3::*)()) &CaBasedCellPopulation3::GetTetrahedralMeshForPdeModifier, 
            " "  )
        .def(
            "GetNode", 
            (::Node<3> *(CaBasedCellPopulation3::*)(unsigned int)) &CaBasedCellPopulation3::GetNode, 
            " " , py::arg("index") )
        .def(
            "GetNumNodes", 
            (unsigned int(CaBasedCellPopulation3::*)()) &CaBasedCellPopulation3::GetNumNodes, 
            " "  )
        .def(
            "GetNeighbouringLocationIndices", 
            (::std::set<unsigned int, std::less<unsigned int>, std::allocator<unsigned int> >(CaBasedCellPopulation3::*)(::CellPtr)) &CaBasedCellPopulation3::GetNeighbouringLocationIndices, 
            " " , py::arg("pCell") )
        .def(
            "GetLocationOfCellCentre", 
            (::boost::numeric::ublas::c_vector<double, 3>(CaBasedCellPopulation3::*)(::CellPtr)) &CaBasedCellPopulation3::GetLocationOfCellCentre, 
            " " , py::arg("pCell") )
        .def(
            "AddCellUsingLocationIndex", 
            (void(CaBasedCellPopulation3::*)(unsigned int, ::CellPtr)) &CaBasedCellPopulation3::AddCellUsingLocationIndex, 
            " " , py::arg("index"), py::arg("pCell") )
        .def(
            "RemoveCellUsingLocationIndex", 
            (void(CaBasedCellPopulation3::*)(unsigned int, ::CellPtr)) &CaBasedCellPopulation3::RemoveCellUsingLocationIndex, 
            " " , py::arg("index"), py::arg("pCell") )
        .def(
            "GetNodeCorrespondingToCell", 
            (::Node<3> *(CaBasedCellPopulation3::*)(::CellPtr)) &CaBasedCellPopulation3::GetNodeCorrespondingToCell, 
            " " , py::arg("pCell") )
        .def(
            "AddCell", 
            (::CellPtr(CaBasedCellPopulation3::*)(::CellPtr, ::CellPtr)) &CaBasedCellPopulation3::AddCell, 
            " " , py::arg("pNewCell"), py::arg("pParentCell") = ::CellPtr( ) )
        .def(
            "EvaluateDivisionPropensity", 
            (double(CaBasedCellPopulation3::*)(unsigned int, unsigned int, ::CellPtr)) &CaBasedCellPopulation3::EvaluateDivisionPropensity, 
            " " , py::arg("currentNodeIndex"), py::arg("targetNodeIndex"), py::arg("pCell") )
        .def(
            "RemoveDeadCells", 
            (unsigned int(CaBasedCellPopulation3::*)()) &CaBasedCellPopulation3::RemoveDeadCells, 
            " "  )
        .def(
            "OpenWritersFiles", 
            (void(CaBasedCellPopulation3::*)(::OutputFileHandler &)) &CaBasedCellPopulation3::OpenWritersFiles, 
            " " , py::arg("rOutputFileHandler") )
        .def(
            "UpdateCellLocations", 
            (void(CaBasedCellPopulation3::*)(double)) &CaBasedCellPopulation3::UpdateCellLocations, 
            " " , py::arg("dt") )
        .def(
            "IsCellAssociatedWithADeletedLocation", 
            (bool(CaBasedCellPopulation3::*)(::CellPtr)) &CaBasedCellPopulation3::IsCellAssociatedWithADeletedLocation, 
            " " , py::arg("pCell") )
        .def(
            "Update", 
            (void(CaBasedCellPopulation3::*)(bool)) &CaBasedCellPopulation3::Update, 
            " " , py::arg("hasHadBirthsOrDeaths") = true )
        .def(
            "AcceptPopulationWriter", 
            (void(CaBasedCellPopulation3::*)(::boost::shared_ptr<AbstractCellPopulationWriter<3, 3> >)) &CaBasedCellPopulation3::AcceptPopulationWriter, 
            " " , py::arg("pPopulationWriter") )
        .def(
            "AcceptPopulationCountWriter", 
            (void(CaBasedCellPopulation3::*)(::boost::shared_ptr<AbstractCellPopulationCountWriter<3, 3> >)) &CaBasedCellPopulation3::AcceptPopulationCountWriter, 
            " " , py::arg("pPopulationCountWriter") )
        .def(
            "AcceptCellWriter", 
            (void(CaBasedCellPopulation3::*)(::boost::shared_ptr<AbstractCellWriter<3, 3> >, ::CellPtr)) &CaBasedCellPopulation3::AcceptCellWriter, 
            " " , py::arg("pCellWriter"), py::arg("pCell") )
        .def(
            "GetVolumeOfCell", 
            (double(CaBasedCellPopulation3::*)(::CellPtr)) &CaBasedCellPopulation3::GetVolumeOfCell, 
            " " , py::arg("pCell") )
        .def(
            "GetWidth", 
            (double(CaBasedCellPopulation3::*)(unsigned int const &)) &CaBasedCellPopulation3::GetWidth, 
            " " , py::arg("rDimension") )
        .def(
            "RemoveAllUpdateRules", 
            (void(CaBasedCellPopulation3::*)()) &CaBasedCellPopulation3::RemoveAllUpdateRules, 
            " "  )
        .def(
            "OutputCellPopulationParameters", 
            (void(CaBasedCellPopulation3::*)(::out_stream &)) &CaBasedCellPopulation3::OutputCellPopulationParameters, 
            " " , py::arg("rParamsFile") )
        .def(
            "IsRoomToDivide", 
            (bool(CaBasedCellPopulation3::*)(::CellPtr)) &CaBasedCellPopulation3::IsRoomToDivide, 
            " " , py::arg("pCell") )
        .def(
            "GetCaBasedDivisionRule", 
            (::boost::shared_ptr<AbstractCaBasedDivisionRule<3> >(CaBasedCellPopulation3::*)()) &CaBasedCellPopulation3::GetCaBasedDivisionRule, 
            " "  )
        .def(
            "SetCaBasedDivisionRule", 
            (void(CaBasedCellPopulation3::*)(::boost::shared_ptr<AbstractCaBasedDivisionRule<3> >)) &CaBasedCellPopulation3::SetCaBasedDivisionRule, 
            " " , py::arg("pCaBasedDivisionRule") )
        .def(
            "AddUpdateRule", 
            (void(CaBasedCellPopulation3::*)(::boost::shared_ptr<AbstractUpdateRule<3> >)) &CaBasedCellPopulation3::AddUpdateRule, 
            " " , py::arg("pUpdateRule") )
        .def(
            "GetUpdateRuleCollection", 
            (::std::vector<boost::shared_ptr<AbstractUpdateRule<3> >, std::allocator<boost::shared_ptr<AbstractUpdateRule<3> > > > const(CaBasedCellPopulation3::*)() const ) &CaBasedCellPopulation3::GetUpdateRuleCollection, 
            " "  )
        .def(
            "GetCellDataItemAtPdeNode", 
            (double(CaBasedCellPopulation3::*)(unsigned int, ::std::string &, bool, double)) &CaBasedCellPopulation3::GetCellDataItemAtPdeNode, 
            " " , py::arg("pdeNodeIndex"), py::arg("rVariableName"), py::arg("dirichletBoundaryConditionApplies") = false, py::arg("dirichletBoundaryValue") = 0. )
        .def(
            "IsPdeNodeAssociatedWithNonApoptoticCell", 
            (bool(CaBasedCellPopulation3::*)(unsigned int)) &CaBasedCellPopulation3::IsPdeNodeAssociatedWithNonApoptoticCell, 
            " " , py::arg("pdeNodeIndex") )
    ;
}
