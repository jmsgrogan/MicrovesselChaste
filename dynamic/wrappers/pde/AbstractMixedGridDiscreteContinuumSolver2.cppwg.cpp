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
#include "AbstractMixedGridDiscreteContinuumSolver.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractMixedGridDiscreteContinuumSolver2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractMixedGridDiscreteContinuumSolver<2 > AbstractMixedGridDiscreteContinuumSolver2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;

class AbstractMixedGridDiscreteContinuumSolver2_Overloads : public AbstractMixedGridDiscreteContinuumSolver2{
    public:
    using AbstractMixedGridDiscreteContinuumSolver2::AbstractMixedGridDiscreteContinuumSolver;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrationsAtCentroids() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
            AbstractMixedGridDiscreteContinuumSolver2,
            GetConcentrationsAtCentroids,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver2,
            Setup,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateElementSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver2,
            UpdateElementSolution,
            rData);
    }
    void UpdateSolution(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver2,
            UpdateCellData,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver2,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractMixedGridDiscreteContinuumSolver2,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver2,
            Write,
            );
    }

};
void register_AbstractMixedGridDiscreteContinuumSolver2_class(py::module &m){
py::class_<AbstractMixedGridDiscreteContinuumSolver2 , AbstractMixedGridDiscreteContinuumSolver2_Overloads , std::shared_ptr<AbstractMixedGridDiscreteContinuumSolver2 >  , AbstractDiscreteContinuumSolver<2>  >(m, "AbstractMixedGridDiscreteContinuumSolver2")
        .def(
            "GetConcentrationsAtCentroids", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractMixedGridDiscreteContinuumSolver2::*)()) &AbstractMixedGridDiscreteContinuumSolver2::GetConcentrationsAtCentroids, 
            " "  )
        .def(
            "Setup", 
            (void(AbstractMixedGridDiscreteContinuumSolver2::*)()) &AbstractMixedGridDiscreteContinuumSolver2::Setup, 
            " "  )
        .def(
            "UpdateSolution", 
            (void(AbstractMixedGridDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractMixedGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateElementSolution", 
            (void(AbstractMixedGridDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractMixedGridDiscreteContinuumSolver2::UpdateElementSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateSolution", 
            (void(AbstractMixedGridDiscreteContinuumSolver2::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const &)) &AbstractMixedGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateCellData", 
            (void(AbstractMixedGridDiscreteContinuumSolver2::*)()) &AbstractMixedGridDiscreteContinuumSolver2::UpdateCellData, 
            " "  )
        .def(
            "Update", 
            (void(AbstractMixedGridDiscreteContinuumSolver2::*)()) &AbstractMixedGridDiscreteContinuumSolver2::Update, 
            " "  )
        .def(
            "Solve", 
            (void(AbstractMixedGridDiscreteContinuumSolver2::*)()) &AbstractMixedGridDiscreteContinuumSolver2::Solve, 
            " "  )
        .def(
            "Write", 
            (void(AbstractMixedGridDiscreteContinuumSolver2::*)()) &AbstractMixedGridDiscreteContinuumSolver2::Write, 
            " "  )
    ;
}
