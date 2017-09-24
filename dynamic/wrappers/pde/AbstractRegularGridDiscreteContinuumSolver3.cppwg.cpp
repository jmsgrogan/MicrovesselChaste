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
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractRegularGridDiscreteContinuumSolver<3 > AbstractRegularGridDiscreteContinuumSolver3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractRegularGridDiscreteContinuumSolver3_Overloads : public AbstractRegularGridDiscreteContinuumSolver3{
    public:
    using AbstractRegularGridDiscreteContinuumSolver3::AbstractRegularGridDiscreteContinuumSolver;
    void Setup() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            Setup,
            );
    }
    void UpdateCellData() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            UpdateCellData,
            );
    }
    void UpdateSolution(::std::vector<double, std::allocator<double> > & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void UpdateSolution(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > & rData) override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            UpdateSolution,
            rData);
    }
    void Update() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            Update,
            );
    }
    void Solve() override {
        PYBIND11_OVERLOAD_PURE(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            Solve,
            );
    }
    void Write() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractRegularGridDiscreteContinuumSolver3,
            Write,
            );
    }

};
void register_AbstractRegularGridDiscreteContinuumSolver3_class(py::module &m){
py::class_<AbstractRegularGridDiscreteContinuumSolver3 , AbstractRegularGridDiscreteContinuumSolver3_Overloads , std::shared_ptr<AbstractRegularGridDiscreteContinuumSolver3 >  , AbstractDiscreteContinuumSolver<3>  >(m, "AbstractRegularGridDiscreteContinuumSolver3")
        .def(
            "Setup", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::Setup, 
            " "  )
        .def(
            "UpdateCellData", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::UpdateCellData, 
            " "  )
        .def(
            "UpdateSolution", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)(::std::vector<double, std::allocator<double> > &)) &AbstractRegularGridDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "UpdateSolution", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)(::std::vector<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> >, std::allocator<RQuantity<std::ratio<0, 1>, std::ratio<-3, 1>, std::ratio<0, 1>, std::ratio<1, 1>, std::ratio<0, 1> > > > &)) &AbstractRegularGridDiscreteContinuumSolver3::UpdateSolution, 
            " " , py::arg("rData") )
        .def(
            "Update", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::Update, 
            " "  )
        .def(
            "Solve", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::Solve, 
            " "  )
        .def(
            "Write", 
            (void(AbstractRegularGridDiscreteContinuumSolver3::*)()) &AbstractRegularGridDiscreteContinuumSolver3::Write, 
            " "  )
    ;
}
