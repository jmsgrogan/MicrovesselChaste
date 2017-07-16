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
#include "AbstractUnstructuredGridDiscreteContinuumSolver.hpp"

#include "AbstractUnstructuredGridDiscreteContinuumSolver3.cppwg.hpp"

namespace py = pybind11;
typedef AbstractUnstructuredGridDiscreteContinuumSolver<3 > AbstractUnstructuredGridDiscreteContinuumSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;

class AbstractUnstructuredGridDiscreteContinuumSolver3_Overloads : public AbstractUnstructuredGridDiscreteContinuumSolver3{
    public:
    using AbstractUnstructuredGridDiscreteContinuumSolver3::AbstractUnstructuredGridDiscreteContinuumSolver;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrationsAtCentroids() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
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
py::class_<AbstractUnstructuredGridDiscreteContinuumSolver3 , AbstractUnstructuredGridDiscreteContinuumSolver3_Overloads , std::shared_ptr<AbstractUnstructuredGridDiscreteContinuumSolver3 >  , AbstractDiscreteContinuumSolver<3>  >(m, "AbstractUnstructuredGridDiscreteContinuumSolver3")
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
