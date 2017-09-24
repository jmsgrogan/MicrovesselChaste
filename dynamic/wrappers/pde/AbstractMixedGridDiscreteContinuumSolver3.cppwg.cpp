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
#include "AbstractMixedGridDiscreteContinuumSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractMixedGridDiscreteContinuumSolver<3 > AbstractMixedGridDiscreteContinuumSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_;

class AbstractMixedGridDiscreteContinuumSolver3_Overloads : public AbstractMixedGridDiscreteContinuumSolver3{
    public:
    using AbstractMixedGridDiscreteContinuumSolver3::AbstractMixedGridDiscreteContinuumSolver;
    ::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > GetConcentrationsAtCentroids() override {
        PYBIND11_OVERLOAD(
            _std_vector_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__std_allocator_lt_RQuantity_lt_std_ratio_lt_0_1_gt__std_ratio_lt_neg3_1_gt__std_ratio_lt_0_1_gt__std_ratio_lt_1_1_gt__std_ratio_lt_0_1_gt__gt__gt__gt_,
            AbstractMixedGridDiscreteContinuumSolver3,
            GetConcentrationsAtCentroids,
            );
    }
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver3,
            Setup,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void UpdateElementSolution(::std::vector<double, std::allocator<double> > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver3,
            UpdateElementSolution,
            rData);
    }
    void UpdateSolution(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver3,
            UpdateCellData,
            );
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver3,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractMixedGridDiscreteContinuumSolver3,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractMixedGridDiscreteContinuumSolver3,
            Write,
            );
    }

};
void register_AbstractMixedGridDiscreteContinuumSolver3_class(py::module &m){
py::class_<AbstractMixedGridDiscreteContinuumSolver3 , AbstractMixedGridDiscreteContinuumSolver3_Overloads , std::shared_ptr<AbstractMixedGridDiscreteContinuumSolver3 >  , AbstractDiscreteContinuumSolver<3>  >(m, "AbstractMixedGridDiscreteContinuumSolver3")
        .def(
            "GetConcentrationsAtCentroids", 
            (::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > >(AbstractMixedGridDiscreteContinuumSolver3::*)()) &AbstractMixedGridDiscreteContinuumSolver3::GetConcentrationsAtCentroids, 
            " "  )
        .def(
            "Setup", 
            (void(AbstractMixedGridDiscreteContinuumSolver3::*)()) &AbstractMixedGridDiscreteContinuumSolver3::Setup, 
            " "  )
        .def(
            "UpdateSolution", 
            (void(AbstractMixedGridDiscreteContinuumSolver3::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractMixedGridDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateElementSolution", 
            (void(AbstractMixedGridDiscreteContinuumSolver3::*)(::std::vector<double, std::allocator<double> > const &)) &AbstractMixedGridDiscreteContinuumSolver3::UpdateElementSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateSolution", 
            (void(AbstractMixedGridDiscreteContinuumSolver3::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > const &)) &AbstractMixedGridDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateCellData", 
            (void(AbstractMixedGridDiscreteContinuumSolver3::*)()) &AbstractMixedGridDiscreteContinuumSolver3::UpdateCellData, 
            " "  )
        .def(
            "Update", 
            (void(AbstractMixedGridDiscreteContinuumSolver3::*)()) &AbstractMixedGridDiscreteContinuumSolver3::Update, 
            " "  )
        .def(
            "Solve", 
            (void(AbstractMixedGridDiscreteContinuumSolver3::*)()) &AbstractMixedGridDiscreteContinuumSolver3::Solve, 
            " "  )
        .def(
            "Write", 
            (void(AbstractMixedGridDiscreteContinuumSolver3::*)()) &AbstractMixedGridDiscreteContinuumSolver3::Write, 
            " "  )
    ;
}
