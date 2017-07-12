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
#include "AbstractDiscreteContinuumPde.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"

#include "AbstractDiscreteContinuumSolver3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractDiscreteContinuumSolver<3 > AbstractDiscreteContinuumSolver3;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > > _std_vectorboost_numeric_ublas_c_vectordouble_3_std_allocatorboost_numeric_ublas_c_vectordouble_3;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::std::vector<double, std::allocator<double> > _std_vectordouble_std_allocatordouble;
typedef ::vtkSmartPointer<vtkDataSet> _vtkSmartPointervtkDataSet;

class AbstractDiscreteContinuumSolver3_Overloads : public AbstractDiscreteContinuumSolver3{
    public:
    using AbstractDiscreteContinuumSolver3::AbstractDiscreteContinuumSolver;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrations() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1,
            AbstractDiscreteContinuumSolver3,
            GetConcentrations,
            );
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrations(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> > pGrid) override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1,
            AbstractDiscreteContinuumSolver3,
            GetConcentrations,
            pGrid);
    }
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrations(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_rationeg3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1,
            AbstractDiscreteContinuumSolver3,
            GetConcentrations,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolution() override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver3,
            GetSolution,
            );
    }
    ::std::vector<double, std::allocator<double> > GetSolution(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver3,
            GetSolution,
            pSamplePoints);
    }
    ::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > > GetSolutionGradients(::vtkSmartPointer<vtkPoints> pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectorboost_numeric_ublas_c_vectordouble_3_std_allocatorboost_numeric_ublas_c_vectordouble_3,
            AbstractDiscreteContinuumSolver3,
            GetSolutionGradients,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolutionP(::vtkPoints * pSamplePoints) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver3,
            GetSolutionP,
            pSamplePoints);
    }
    ::std::vector<double, std::allocator<double> > GetSolution(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> > pGrid) override {
        PYBIND11_OVERLOAD(
            _std_vectordouble_std_allocatordouble,
            AbstractDiscreteContinuumSolver3,
            GetSolution,
            pGrid);
    }
    ::vtkSmartPointer<vtkDataSet> GetVtkSolution() override {
        PYBIND11_OVERLOAD(
            _vtkSmartPointervtkDataSet,
            AbstractDiscreteContinuumSolver3,
            GetVtkSolution,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver3,
            Setup,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver3,
            Solve,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver3,
            Update,
            );
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver3,
            UpdateCellData,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void UpdateSolution(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void Write() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractDiscreteContinuumSolver3,
            Write,
            );
    }

};
void register_AbstractDiscreteContinuumSolver3_class(py::module &m){
py::class_<AbstractDiscreteContinuumSolver3 , AbstractDiscreteContinuumSolver3_Overloads   >(m, "AbstractDiscreteContinuumSolver3")
        .def(
            "AddBoundaryCondition", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::shared_ptr<DiscreteContinuumBoundaryCondition<3> >)) &AbstractDiscreteContinuumSolver3::AddBoundaryCondition, 
            " " , py::arg("pBoundaryCondition") )
        .def(
            "GetConcentrations", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::GetConcentrations, 
            " "  )
        .def(
            "GetConcentrations", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractDiscreteContinuumSolver3::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >)) &AbstractDiscreteContinuumSolver3::GetConcentrations, 
            " " , py::arg("pGrid") )
        .def(
            "GetConcentrations", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractDiscreteContinuumSolver3::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver3::GetConcentrations, 
            " " , py::arg("pSamplePoints") )
        .def(
            "GetDensityMap", 
            (::std::shared_ptr<DensityMap<3> >(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::GetDensityMap, 
            " "  )
        .def(
            "GetLabel", 
            (::std::string const &(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::GetLabel, 
            " "  )
        .def(
            "GetPde", 
            (::std::shared_ptr<AbstractDiscreteContinuumPde<3, 3> >(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::GetPde, 
            " "  )
        .def(
            "GetReferenceLength", 
            (::QLength(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::GetReferenceLength, 
            " "  )
        .def(
            "GetReferenceConcentration", 
            (::QConcentration(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::GetReferenceConcentration, 
            " "  )
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::GetSolution, 
            " "  )
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver3::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver3::GetSolution, 
            " " , py::arg("pSamplePoints") )
        .def(
            "GetSolutionGradients", 
            (::std::vector<boost::numeric::ublas::c_vector<double, 3>, std::allocator<boost::numeric::ublas::c_vector<double, 3> > >(AbstractDiscreteContinuumSolver3::*)(::vtkSmartPointer<vtkPoints>)) &AbstractDiscreteContinuumSolver3::GetSolutionGradients, 
            " " , py::arg("pSamplePoints") )
        .def(
            "GetSolutionP", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver3::*)(::vtkPoints *)) &AbstractDiscreteContinuumSolver3::GetSolutionP, 
            " " , py::arg("pSamplePoints") )
        .def(
            "GetSolution", 
            (::std::vector<double, std::allocator<double> >(AbstractDiscreteContinuumSolver3::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >)) &AbstractDiscreteContinuumSolver3::GetSolution, 
            " " , py::arg("pGrid") )
        .def(
            "GetVtkSolution", 
            (::vtkSmartPointer<vtkDataSet>(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::GetVtkSolution, 
            " "  )
        .def(
            "SetFileHandler", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::shared_ptr<OutputFileHandler>)) &AbstractDiscreteContinuumSolver3::SetFileHandler, 
            " " , py::arg("pOutputFileHandler") )
        .def(
            "SetFileName", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::string const &)) &AbstractDiscreteContinuumSolver3::SetFileName, 
            " " , py::arg("rFilename") )
        .def(
            "SetLabel", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::string const &)) &AbstractDiscreteContinuumSolver3::SetLabel, 
            " " , py::arg("rLabel") )
        .def(
            "SetPde", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::shared_ptr<AbstractDiscreteContinuumPde<3, 3> >)) &AbstractDiscreteContinuumSolver3::SetPde, 
            " " , py::arg("pPde") )
        .def(
            "Setup", 
            (void(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::Setup, 
            " "  )
        .def(
            "SetReferenceConcentration", 
            (void(AbstractDiscreteContinuumSolver3::*)(::QConcentration)) &AbstractDiscreteContinuumSolver3::SetReferenceConcentration, 
            " " , py::arg("referenceConcentration") )
        .def(
            "SetWriteSolution", 
            (void(AbstractDiscreteContinuumSolver3::*)(bool)) &AbstractDiscreteContinuumSolver3::SetWriteSolution, 
            " " , py::arg("write") = true )
        .def(
            "SetGrid", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::shared_ptr<AbstractDiscreteContinuumGrid<3, 3> >)) &AbstractDiscreteContinuumSolver3::SetGrid, 
            " " , py::arg("pGrid") )
        .def(
            "SetDensityMap", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::shared_ptr<DensityMap<3> >)) &AbstractDiscreteContinuumSolver3::SetDensityMap, 
            " " , py::arg("pDensityMap") )
        .def(
            "SetVesselNetwork", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::shared_ptr<VesselNetwork<3> >)) &AbstractDiscreteContinuumSolver3::SetVesselNetwork, 
            " " , py::arg("pNetwork") )
        .def(
            "SetCellPopulation", 
            (void(AbstractDiscreteContinuumSolver3::*)(::AbstractCellPopulation<3, 3> &, ::QLength, ::QConcentration)) &AbstractDiscreteContinuumSolver3::SetCellPopulation, 
            " " , py::arg("rCellPopulation"), py::arg("cellPopulationReferenceLength"), py::arg("cellPopulationReferenceConcentration") )
        .def(
            "Solve", 
            (void(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::Solve, 
            " "  )
        .def(
            "Update", 
            (void(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::Update, 
            " "  )
        .def(
            "UpdateCellData", 
            (void(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::UpdateCellData, 
            " "  )
        .def(
            "UpdateSolution", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateSolution", 
            (void(AbstractDiscreteContinuumSolver3::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const &)) &AbstractDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "Write", 
            (void(AbstractDiscreteContinuumSolver3::*)()) &AbstractDiscreteContinuumSolver3::Write, 
            " "  )
    ;
}
