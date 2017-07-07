#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <set>
#include <vector>
#include <string>
#include <map>
#include "SmartPointers.hpp"
#include "UblasIncludes.hpp"
#include "UnitCollection.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"

#include "CoupledVegfPelletDiffusionReactionPde2_2.cppwg.hpp"

namespace py = pybind11;
typedef CoupledVegfPelletDiffusionReactionPde<2,2 > CoupledVegfPelletDiffusionReactionPde2_2;
;
typedef ::QConcentrationFlowRate _QConcentrationFlowRate;
typedef ::QRate _QRate;

class CoupledVegfPelletDiffusionReactionPde2_2_Overloads : public CoupledVegfPelletDiffusionReactionPde2_2{
    public:
    using CoupledVegfPelletDiffusionReactionPde2_2::CoupledVegfPelletDiffusionReactionPde;
    double ComputeSourceTerm(::ChastePoint<2> const & rX, double u, ::Element<2, 2> * pElement) override {
        PYBIND11_OVERLOAD(
            double,
            CoupledVegfPelletDiffusionReactionPde2_2,
            ComputeSourceTerm,
            rX, 
u, 
pElement);
    }
    ::QConcentrationFlowRate ComputeSourceTerm(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QConcentrationFlowRate,
            CoupledVegfPelletDiffusionReactionPde2_2,
            ComputeSourceTerm,
            gridIndex, 
u);
    }
    ::QRate ComputeSourceTermPrime(unsigned int gridIndex, ::QConcentration u) override {
        PYBIND11_OVERLOAD(
            _QRate,
            CoupledVegfPelletDiffusionReactionPde2_2,
            ComputeSourceTermPrime,
            gridIndex, 
u);
    }
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledVegfPelletDiffusionReactionPde2_2,
            UpdateDiscreteSourceStrengths,
            );
    }
    void UpdateMultiplierValue() override {
        PYBIND11_OVERLOAD(
            void,
            CoupledVegfPelletDiffusionReactionPde2_2,
            UpdateMultiplierValue,
            );
    }

};
void register_CoupledVegfPelletDiffusionReactionPde2_2_class(py::module &m){
py::class_<CoupledVegfPelletDiffusionReactionPde2_2 , CoupledVegfPelletDiffusionReactionPde2_2_Overloads   >(m, "CoupledVegfPelletDiffusionReactionPde2_2")
        .def(py::init< >())
        .def_static(
            "Create", 
            (::std::shared_ptr<CoupledVegfPelletDiffusionReactionPde<2, 2> >(*)()) &CoupledVegfPelletDiffusionReactionPde2_2::Create, 
            " "  )
        .def(
            "ComputeSourceTerm", 
            (double(CoupledVegfPelletDiffusionReactionPde2_2::*)(::ChastePoint<2> const &, double, ::Element<2, 2> *)) &CoupledVegfPelletDiffusionReactionPde2_2::ComputeSourceTerm, 
            " " , py::arg("rX"), py::arg("u"), py::arg("pElement") = __null )
        .def(
            "ComputeSourceTerm", 
            (::QConcentrationFlowRate(CoupledVegfPelletDiffusionReactionPde2_2::*)(unsigned int, ::QConcentration)) &CoupledVegfPelletDiffusionReactionPde2_2::ComputeSourceTerm, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "ComputeSourceTermPrime", 
            (::QRate(CoupledVegfPelletDiffusionReactionPde2_2::*)(unsigned int, ::QConcentration)) &CoupledVegfPelletDiffusionReactionPde2_2::ComputeSourceTermPrime, 
            " " , py::arg("gridIndex"), py::arg("u") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::UpdateDiscreteSourceStrengths, 
            " "  )
        .def(
            "UpdateMultiplierValue", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::UpdateMultiplierValue, 
            " "  )
        .def(
            "GetPelletFreeDecayRate", 
            (::QRate(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::GetPelletFreeDecayRate, 
            " "  )
        .def(
            "GetPelletBindingConstant", 
            (::QDimensionless(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::GetPelletBindingConstant, 
            " "  )
        .def(
            "GetInitialVegfInPellet", 
            (::QConcentration(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::GetInitialVegfInPellet, 
            " "  )
        .def(
            "GetCurrentVegfInPellet", 
            (::QConcentration(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::GetCurrentVegfInPellet, 
            " "  )
        .def(
            "GetCorneaPelletPermeability", 
            (::QMembranePermeability(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::GetCorneaPelletPermeability, 
            " "  )
        .def(
            "GetPelletSurfaceArea", 
            (::QArea(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::GetPelletSurfaceArea, 
            " "  )
        .def(
            "GetPelletDepth", 
            (::QLength(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::GetPelletDepth, 
            " "  )
        .def(
            "GetPelletVolume", 
            (::QVolume(CoupledVegfPelletDiffusionReactionPde2_2::*)()) &CoupledVegfPelletDiffusionReactionPde2_2::GetPelletVolume, 
            " "  )
        .def(
            "SetPelletFreeDecayRate", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QRate)) &CoupledVegfPelletDiffusionReactionPde2_2::SetPelletFreeDecayRate, 
            " " , py::arg("rate") )
        .def(
            "SetPelletBindingConstant", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QDimensionless)) &CoupledVegfPelletDiffusionReactionPde2_2::SetPelletBindingConstant, 
            " " , py::arg("bindingConstant") )
        .def(
            "SetInitialVegfInPellet", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QConcentration)) &CoupledVegfPelletDiffusionReactionPde2_2::SetInitialVegfInPellet, 
            " " , py::arg("initialVegf") )
        .def(
            "SetCurrentVegfInPellet", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QConcentration)) &CoupledVegfPelletDiffusionReactionPde2_2::SetCurrentVegfInPellet, 
            " " , py::arg("currentVegf") )
        .def(
            "SetCorneaPelletPermeability", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QMembranePermeability)) &CoupledVegfPelletDiffusionReactionPde2_2::SetCorneaPelletPermeability, 
            " " , py::arg("permeability") )
        .def(
            "SetPelletSurfaceArea", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QArea)) &CoupledVegfPelletDiffusionReactionPde2_2::SetPelletSurfaceArea, 
            " " , py::arg("surfaceArea") )
        .def(
            "SetPelletDepth", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QLength)) &CoupledVegfPelletDiffusionReactionPde2_2::SetPelletDepth, 
            " " , py::arg("depth") )
        .def(
            "SetPelletVolume", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QVolume)) &CoupledVegfPelletDiffusionReactionPde2_2::SetPelletVolume, 
            " " , py::arg("volume") )
        .def(
            "SetHalfMaxVegfConcentration", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QConcentration)) &CoupledVegfPelletDiffusionReactionPde2_2::SetHalfMaxVegfConcentration, 
            " " , py::arg("halfMax") )
        .def(
            "SetContinuumLinearInUTerm", 
            (void(CoupledVegfPelletDiffusionReactionPde2_2::*)(::QRate)) &CoupledVegfPelletDiffusionReactionPde2_2::SetContinuumLinearInUTerm, 
            " " , py::arg("linearInUTerm") )
    ;
}
