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
#include "CoupledVegfPelletDiffusionReactionPde.hpp"

#include "PythonObjectConverters.hpp"
#include "CoupledVegfPelletDiffusionReactionPde3_3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef CoupledVegfPelletDiffusionReactionPde<3,3 > CoupledVegfPelletDiffusionReactionPde3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class CoupledVegfPelletDiffusionReactionPde3_3_Overloads : public CoupledVegfPelletDiffusionReactionPde3_3{
    public:
    using CoupledVegfPelletDiffusionReactionPde3_3::CoupledVegfPelletDiffusionReactionPde;
    double ComputeSourceTerm(::ChastePoint<3> const & rX, double u, ::Element<3, 3> * pElement) override {
        PYBIND11_OVERLOAD(
            double,
            CoupledVegfPelletDiffusionReactionPde3_3,
            ComputeSourceTerm,
            rX, 
u, 
pElement);
    }
    ::QConcentrationFlowRate ComputeSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            CoupledVegfPelletDiffusionReactionPde3_3,
            ComputeSourceTerm,
            gridIndex, 
u);
    }
    ::QRate ComputeSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QRate,
            CoupledVegfPelletDiffusionReactionPde3_3,
            ComputeSourceTermPrime,
            gridIndex, 
u);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledVegfPelletDiffusionReactionPde3_3,
            UpdateDiscreteSourceStrengths,
            );
    }
    void UpdateMultiplierValue() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledVegfPelletDiffusionReactionPde3_3,
            UpdateMultiplierValue,
            );
    }

};
void register_CoupledVegfPelletDiffusionReactionPde3_3_class(py::module &m){
py::class_<CoupledVegfPelletDiffusionReactionPde3_3 , CoupledVegfPelletDiffusionReactionPde3_3_Overloads , std::shared_ptr<CoupledVegfPelletDiffusionReactionPde3_3 >  , AbstractDiscreteContinuumParabolicPde<3, 3>  >(m, "CoupledVegfPelletDiffusionReactionPde3_3")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CoupledVegfPelletDiffusionReactionPde<3, 3> >(*)()) &CoupledVegfPelletDiffusionReactionPde3_3::Create, 
            " "  )
        .def(
            "ComputeSourceTerm", 
            (double(CoupledVegfPelletDiffusionReactionPde3_3::*)(::ChastePoint<3> const &, double, ::Element<3, 3> *)) &CoupledVegfPelletDiffusionReactionPde3_3::ComputeSourceTerm, 
            " " , py::arg("rX"), py::arg("u"), py::arg("pElement") = __null )
        .def(
            "ComputeSourceTerm", 
            (::QConcentrationFlowRate(CoupledVegfPelletDiffusionReactionPde3_3::*)(unsigned int, ::QConcentration)) &CoupledVegfPelletDiffusionReactionPde3_3::ComputeSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeSourceTermPrime", 
            (::QRate(CoupledVegfPelletDiffusionReactionPde3_3::*)(unsigned int, ::QConcentration)) &CoupledVegfPelletDiffusionReactionPde3_3::ComputeSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::UpdateDiscreteSourceStrengths, 
            " "  )
        .def(
            "UpdateMultiplierValue", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::UpdateMultiplierValue, 
            " "  )
        .def(
            "GetPelletFreeDecayRate", 
            (::QRate(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::GetPelletFreeDecayRate, 
            " "  )
        .def(
            "GetPelletBindingConstant", 
            (::QDimensionless(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::GetPelletBindingConstant, 
            " "  )
        .def(
            "GetInitialVegfInPellet", 
            (::QConcentration(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::GetInitialVegfInPellet, 
            " "  )
        .def(
            "GetCurrentVegfInPellet", 
            (::QConcentration(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::GetCurrentVegfInPellet, 
            " "  )
        .def(
            "GetCorneaPelletPermeability", 
            (::QMembranePermeability(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::GetCorneaPelletPermeability, 
            " "  )
        .def(
            "GetPelletSurfaceArea", 
            (::QArea(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::GetPelletSurfaceArea, 
            " "  )
        .def(
            "GetPelletDepth", 
            (::QLength(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::GetPelletDepth, 
            " "  )
        .def(
            "GetPelletVolume", 
            (::QVolume(CoupledVegfPelletDiffusionReactionPde3_3::*)()) &CoupledVegfPelletDiffusionReactionPde3_3::GetPelletVolume, 
            " "  )
        .def(
            "SetPelletFreeDecayRate", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QRate)) &CoupledVegfPelletDiffusionReactionPde3_3::SetPelletFreeDecayRate, 
            " " , py::arg("rate") )
        .def(
            "SetPelletBindingConstant", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QDimensionless)) &CoupledVegfPelletDiffusionReactionPde3_3::SetPelletBindingConstant, 
            " " , py::arg("bindingConstant") )
        .def(
            "SetInitialVegfInPellet", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QConcentration)) &CoupledVegfPelletDiffusionReactionPde3_3::SetInitialVegfInPellet, 
            " " , py::arg("initialVegf") )
        .def(
            "SetCurrentVegfInPellet", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QConcentration)) &CoupledVegfPelletDiffusionReactionPde3_3::SetCurrentVegfInPellet, 
            " " , py::arg("currentVegf") )
        .def(
            "SetCorneaPelletPermeability", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QMembranePermeability)) &CoupledVegfPelletDiffusionReactionPde3_3::SetCorneaPelletPermeability, 
            " " , py::arg("permeability") )
        .def(
            "SetPelletSurfaceArea", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QArea)) &CoupledVegfPelletDiffusionReactionPde3_3::SetPelletSurfaceArea, 
            " " , py::arg("surfaceArea") )
        .def(
            "SetPelletDepth", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QLength)) &CoupledVegfPelletDiffusionReactionPde3_3::SetPelletDepth, 
            " " , py::arg("depth") )
        .def(
            "SetPelletVolume", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QVolume)) &CoupledVegfPelletDiffusionReactionPde3_3::SetPelletVolume, 
            " " , py::arg("volume") )
        .def(
            "SetHalfMaxVegfConcentration", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QConcentration)) &CoupledVegfPelletDiffusionReactionPde3_3::SetHalfMaxVegfConcentration, 
            " " , py::arg("halfMax") )
        .def(
            "SetContinuumLinearInUTerm", 
            (void(CoupledVegfPelletDiffusionReactionPde3_3::*)(::QRate)) &CoupledVegfPelletDiffusionReactionPde3_3::SetContinuumLinearInUTerm, 
            " " , py::arg("linearInUTerm") )
    ;
}
