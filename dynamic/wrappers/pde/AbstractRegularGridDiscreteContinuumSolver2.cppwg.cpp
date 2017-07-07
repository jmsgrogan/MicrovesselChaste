#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"

#include "AbstractRegularGridDiscreteContinuumSolver2.cppwg.hpp"

namespace py = pybind11;
typedef AbstractRegularGridDiscreteContinuumSolver<2 > AbstractRegularGridDiscreteContinuumSolver2;
;

class AbstractRegularGridDiscreteContinuumSolver2_Overloads : public AbstractRegularGridDiscreteContinuumSolver2{
    public:
    using AbstractRegularGridDiscreteContinuumSolver2::AbstractRegularGridDiscreteContinuumSolver;
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            Setup,
            );
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            UpdateCellData,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void UpdateSolution(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            UpdateSolution,
            rData);
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver2,
            Write,
            );
    }

};
void register_AbstractRegularGridDiscreteContinuumSolver2_class(py::module &m){
py::class_<AbstractRegularGridDiscreteContinuumSolver2 , AbstractRegularGridDiscreteContinuumSolver2_Overloads   >(m, "AbstractRegularGridDiscreteContinuumSolver2")
        .def(
            "Setup", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::Setup, 
            " "  )
        .def(
            "UpdateCellData", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::UpdateCellData, 
            " "  )
        .def(
            "UpdateSolution", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)(::std::vector<double, std::allocator<double> > &)) &AbstractRegularGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateSolution", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > &)) &AbstractRegularGridDiscreteContinuumSolver2::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "Update", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::Update, 
            " "  )
        .def(
            "Solve", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::Solve, 
            " "  )
        .def(
            "Write", 
            (void(AbstractRegularGridDiscreteContinuumSolver2::*)()) &AbstractRegularGridDiscreteContinuumSolver2::Write, 
            " "  )
    ;
}
