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

#include "AbstractUnstructuredGridDiscreteContinuumSolver3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractUnstructuredGridDiscreteContinuumSolver<3 > AbstractUnstructuredGridDiscreteContinuumSolver3;
;
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1;

class AbstractUnstructuredGridDiscreteContinuumSolver3_Overloads : public AbstractUnstructuredGridDiscreteContinuumSolver3{
    public:
    using AbstractUnstructuredGridDiscreteContinuumSolver3::AbstractUnstructuredGridDiscreteContinuumSolver;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrationsAtCentroids() override {
        PYBIND11_OVERLOAD(
            _std_vectorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1_std_allocatorRQuantitystd_ratio0_1_std_ratio-3_1_std_ratio0_1_std_ratio1_1_std_ratio0_1,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            GetConcentrationsAtCentroids,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            Setup,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void UpdateElementSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            UpdateElementSolution,
            rData);
    }
    void UpdateSolution(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            UpdateCellData,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractUnstructuredGridDiscreteContinuumSolver3,
            Write,
            );
    }

};
void register_AbstractUnstructuredGridDiscreteContinuumSolver3_class(py::module &m){
py::class_<AbstractUnstructuredGridDiscreteContinuumSolver3 , AbstractUnstructuredGridDiscreteContinuumSolver3_Overloads   >(m, "AbstractUnstructuredGridDiscreteContinuumSolver3")
        .def(
            "GetConcentrationsAtCentroids", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractUnstructuredGridDiscreteContinuumSolver3::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver3::GetConcentrationsAtCentroids, 
            " "  )
        .def(
            "Setup", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver3::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver3::Setup, 
            " "  )
        .def(
            "UpdateSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver3::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateElementSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver3::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver3::UpdateElementSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateSolution", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver3::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const &)) &AbstractUnstructuredGridDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateCellData", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver3::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver3::UpdateCellData, 
            " "  )
        .def(
            "Update", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver3::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver3::Update, 
            " "  )
        .def(
            "Solve", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver3::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver3::Solve, 
            " "  )
        .def(
            "Write", 
            (void(AbstractUnstructuredGridDiscreteContinuumSolver3::*)()) &AbstractUnstructuredGridDiscreteContinuumSolver3::Write, 
            " "  )
    ;
}
