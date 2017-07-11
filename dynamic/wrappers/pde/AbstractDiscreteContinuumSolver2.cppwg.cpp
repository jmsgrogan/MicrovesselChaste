#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"

#include "AbstractDiscreteContinuumSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumSolver<2 > AbstractDiscreteContinuumSolver2;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > > _std_vectorboost_numeric_ublas_c_vectordouble_3_std_allocatorboost_numeric_ublas_c_vectordouble_3;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::vtkSmartPointer<vtkDataSet> _vtkSmartPointervtkDataSet;

class AbstractDiscreteContinuumSolver2_Overloads : public AbstractDiscreteContinuumSolver2{
    public:
    using AbstractDiscreteContinuumSolver2::AbstractDiscreteContinuumSolver;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrations() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1,
            AbstractDiscreteContinuumSolver2,
            GetConcentrations,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrations(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> > pGrid) override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1,
            AbstractDiscreteContinuumSolver2,
            GetConcentrations,
            pGrid);
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrations(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1,
            AbstractDiscreteContinuumSolver2,
            GetConcentrations,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolution() override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver2,
            GetSolution,
            );
    }
    ::std::vector<double, std::allocator<double> > GetSolution(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver2,
            GetSolution,
            pSamplePoints);
    }
    ::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > > GetSolutionGradients(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectorboost_numeric_ublas_c_vectordouble_3_std_allocatorboost_numeric_ublas_c_vectordouble_3,
            AbstractDiscreteContinuumSolver2,
            GetSolutionGradients,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolutionP(::vtkPoints * pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver2,
            GetSolutionP,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolution(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> > pGrid) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver2,
            GetSolution,
            pGrid);
    }
    ::vtkSmartPointer<vtkDataSet> GetVtkSolution() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointervtkDataSet,
            AbstractDiscreteContinuumSolver2,
            GetVtkSolution,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            Setup,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            Solve,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            Update,
            );
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            UpdateCellData,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateSolution(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void Write() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver2,
            Write,
            );
    }

};
void register_AbstractDiscreteContinuumSolver2_class(py::module &m){
py::class_<AbstractDiscreteContinuumSolver2 , AbstractDiscreteContinuumSolver2_Overloads   >(m, "AbstractDiscreteContinuumSolver2")
        .def(
            "AddBoundaryCondition", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<DiscreteContinuumBoundaryCondition<2> >)) &AbstractDiscreteContinuumSolver2::AddBoundaryCondition, 
            " " , py::arg("pBoundaryCondition") )
        .def(
            "GetConcentrations", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetConcentrations, 
            " "  )
        .def(
            "GetConcentrations", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &AbstractDiscreteContinuumSolver2::GetConcentrations, 
            " " , py::arg("pGrid") )
        .def(
            "GetConcentrations", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractDiscreteContinuumSolver2::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver2::GetConcentrations, 
            " " , py::arg("pSamplePoints") )
        .def(
            "GetDensityMap", 
            (::std::shared_ptr<DensityMap<2> >(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetDensityMap, 
            " "  )
        .def(
            "GetLabel", 
            (::std::string const &(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetLabel, 
            " "  )
        .def(
            "GetPde", 
            (::std::shared_ptr<AbstractDiscreteContinuumPde<2, 2> >(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetPde, 
            " "  )
        .def(
            "GetReferenceLength", 
            (::QLength(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetReferenceLength, 
            " "  )
        .def(
            "GetReferenceConcentration", 
            (::QConcentration(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetReferenceConcentration, 
            " "  )
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetSolution, 
            " "  )
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver2::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver2::GetSolution, 
            " " , py::arg("pSamplePoints") )
        .def(
            "GetSolutionGradients", 
            (::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > >(AbstractDiscreteContinuumSolver2::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver2::GetSolutionGradients, 
            " " , py::arg("pSamplePoints") )
        .def(
            "GetSolutionP", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver2::*)(::vtkPoints *)) &AbstractDiscreteContinuumSolver2::GetSolutionP, 
            " " , py::arg("pSamplePoints") )
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &AbstractDiscreteContinuumSolver2::GetSolution, 
            " " , py::arg("pGrid") )
        .def(
            "GetVtkSolution", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::GetVtkSolution, 
            " "  )
        .def(
            "SetFileHandler", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<OutputFileHandler>)) &AbstractDiscreteContinuumSolver2::SetFileHandler, 
            " " , py::arg("pOutputFileHandler") )
        .def(
            "SetFileName", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::string const &)) &AbstractDiscreteContinuumSolver2::SetFileName, 
            " " , py::arg("rFilename") )
        .def(
            "SetLabel", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::string const &)) &AbstractDiscreteContinuumSolver2::SetLabel, 
            " " , py::arg("rLabel") )
        .def(
            "SetPde", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumPde<2, 2> >)) &AbstractDiscreteContinuumSolver2::SetPde, 
            " " , py::arg("pPde") )
        .def(
            "Setup", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::Setup, 
            " "  )
        .def(
            "SetReferenceConcentration", 
            (void(AbstractDiscreteContinuumSolver2::*)(::QConcentration)) &AbstractDiscreteContinuumSolver2::SetReferenceConcentration, 
            " " , py::arg("referenceConcentration") )
        .def(
            "SetWriteSolution", 
            (void(AbstractDiscreteContinuumSolver2::*)(bool)) &AbstractDiscreteContinuumSolver2::SetWriteSolution, 
            " " , py::arg("write") = true )
        .def(
            "SetGrid", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<2, 2> >)) &AbstractDiscreteContinuumSolver2::SetGrid, 
            " " , py::arg("pGrid") )
        .def(
            "SetDensityMap", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<DensityMap<2> >)) &AbstractDiscreteContinuumSolver2::SetDensityMap, 
            " " , py::arg("pDensityMap") )
        .def(
            "SetVesselNetwork", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::shared_ptr<VesselNetwork<2> >)) &AbstractDiscreteContinuumSolver2::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetCellPopulation", 
            (void(AbstractDiscreteContinuumSolver2::*)(::AbstractCellPopulation<2, 2> &, ::QLength, ::QConcentration)) &AbstractDiscreteContinuumSolver2::SetCellPopulation, 
            " " , py::arg("rCellPopulation"), py::arg("cellPopulationReferenceLength"), py::arg("cellPopulationReferenceConcentration") )
        .def(
            "Solve", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::Solve, 
            " "  )
        .def(
            "Update", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::Update, 
            " "  )
        .def(
            "UpdateCellData", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::UpdateCellData, 
            " "  )
        .def(
            "UpdateSolution", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateSolution", 
            (void(AbstractDiscreteContinuumSolver2::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const &)) &AbstractDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "Write", 
            (void(AbstractDiscreteContinuumSolver2::*)()) &AbstractDiscreteContinuumSolver2::Write, 
            " "  )
    ;
}