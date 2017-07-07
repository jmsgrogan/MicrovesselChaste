#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractUnstructuredGridDiscreteContinuumSolver.hpp"

#include "AbstractUnstructuredGridDiscreteContinuumSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractUnstructuredGridDiscreteContinuumSolver<2 > AbstractUnstructuredGridDiscreteContinuumSolver2;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1;

class AbstractUnstructuredGridDiscreteContinuumSolver2_Overloads : public AbstractUnstructuredGridDiscreteContinuumSolver2{
    public:
    using AbstractUnstructuredGridDiscreteContinuumSolver2::AbstractUnstructuredGridDiscreteContinuumSolver;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrationsAtCentroids() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            GetConcentrationsAtCentroids,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            Setup,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateElementSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            UpdateElementSolution,
            rData);
    }
    void UpdateSolution(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            UpdateCellData,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver2,
            Write,
            );
    }

};
void register_AbstractUnstructuredGridDiscreteContinuumSolver2_class(py::module &m){
py::class_<AbstractUnstructuredGridDiscreteContinuumSolver2 , AbstractUnstructuredGridDiscreteContinuumSolver2_Overloads   >(m, "AbstractUnstructuredGridDiscreteContinuumSolver2")
        .def(
            "GetConcentrationsAtCentroids", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::GetConcentrationsAtCentroids, 
            " "  )
        .def(
            "Setup", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::Setup, 
            " "  )
        .def(
            "UpdateSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateElementSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver2::UpdateElementSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateCellData", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::UpdateCellData, 
            " "  )
        .def(
            "Update", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::Update, 
            " "  )
        .def(
            "Solve", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::Solve, 
            " "  )
        .def(
            "Write", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver2::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver2::Write, 
            " "  )
    ;
}
