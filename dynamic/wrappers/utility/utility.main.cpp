#include <pybind11/pybind11.h>
#include "UnitCollection.hpp"
#include "VectorUnitCollection.hpp"
#include <pybind11/operators.h>
#include "UblasIncludes.hpp"
#include "PythonObjectConverters.hpp"
PYBIND11_CVECTOR_TYPECASTER3();
PYBIND11_CVECTOR_TYPECASTER2();
#include "BaseUnits.cppwg.hpp"
#include "ParameterCollection.cppwg.hpp"
#include "Owen11Parameters.cppwg.hpp"
#include "Connor17Parameters.cppwg.hpp"
#include "Secomb04Parameters.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_utility, m)
{
    register_BaseUnits_class(m);
    register_ParameterCollection_class(m);
    register_Owen11Parameters_class(m);
    register_Connor17Parameters_class(m);
    register_Secomb04Parameters_class(m);
        py::class_<VecQLength<2> >(m, "VecQLength2")
        .def(py::init< >())
        .def(py::init<c_vector<double, 2>, QLength >())
        .def(py::init<double >())
        .def("Convert", 
            (c_vector<double, 2>(VecQLength<2>::*)(const QLength& rhs) const) &VecQLength<2>::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (c_vector<double, 2>(VecQLength<2>::*)()) &VecQLength<2>::GetValue, 
            " " )
    ;
        py::class_<VecQLength<3> >(m, "VecQLength3")
        .def(py::init< >())
        .def(py::init<c_vector<double, 3>, QLength >())
        .def(py::init<double >())
        .def("Convert", 
            (c_vector<double, 3>(VecQLength<3>::*)(const QLength& rhs) const) &VecQLength<3>::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (c_vector<double, 3>(VecQLength<3>::*)()) &VecQLength<3>::GetValue, 
            " " )
    ;
        py::class_<QVolumetricSolubility>(m, "QVolumetricSolubility")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QVolumetricSolubility::*)(const QVolumetricSolubility& rhs) const) &QVolumetricSolubility::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QVolumetricSolubility::*)()) &QVolumetricSolubility::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QSolubility>(m, "QSolubility")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QSolubility::*)(const QSolubility& rhs) const) &QSolubility::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QSolubility::*)()) &QSolubility::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QDiffusivityPerConcentration>(m, "QDiffusivityPerConcentration")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QDiffusivityPerConcentration::*)(const QDiffusivityPerConcentration& rhs) const) &QDiffusivityPerConcentration::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QDiffusivityPerConcentration::*)()) &QDiffusivityPerConcentration::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QDiffusivity>(m, "QDiffusivity")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QDiffusivity::*)(const QDiffusivity& rhs) const) &QDiffusivity::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QDiffusivity::*)()) &QDiffusivity::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QFlowRate>(m, "QFlowRate")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QFlowRate::*)(const QFlowRate& rhs) const) &QFlowRate::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QFlowRate::*)()) &QFlowRate::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QFlowImpedance>(m, "QFlowImpedance")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QFlowImpedance::*)(const QFlowImpedance& rhs) const) &QFlowImpedance::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QFlowImpedance::*)()) &QFlowImpedance::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QDynamicViscosity>(m, "QDynamicViscosity")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QDynamicViscosity::*)(const QDynamicViscosity& rhs) const) &QDynamicViscosity::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QDynamicViscosity::*)()) &QDynamicViscosity::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QPressure>(m, "QPressure")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QPressure::*)(const QPressure& rhs) const) &QPressure::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QPressure::*)()) &QPressure::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QForce>(m, "QForce")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QForce::*)(const QForce& rhs) const) &QForce::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QForce::*)()) &QForce::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QVelocity>(m, "QVelocity")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QVelocity::*)(const QVelocity& rhs) const) &QVelocity::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QVelocity::*)()) &QVelocity::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QNumberDensity>(m, "QNumberDensity")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QNumberDensity::*)(const QNumberDensity& rhs) const) &QNumberDensity::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QNumberDensity::*)()) &QNumberDensity::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QMolarMass>(m, "QMolarMass")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QMolarMass::*)(const QMolarMass& rhs) const) &QMolarMass::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QMolarMass::*)()) &QMolarMass::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QRatePerConcentration>(m, "QRatePerConcentration")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QRatePerConcentration::*)(const QRatePerConcentration& rhs) const) &QRatePerConcentration::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QRatePerConcentration::*)()) &QRatePerConcentration::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QConcentrationGradient>(m, "QConcentrationGradient")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QConcentrationGradient::*)(const QConcentrationGradient& rhs) const) &QConcentrationGradient::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QConcentrationGradient::*)()) &QConcentrationGradient::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QConcentrationFlux>(m, "QConcentrationFlux")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QConcentrationFlux::*)(const QConcentrationFlux& rhs) const) &QConcentrationFlux::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QConcentrationFlux::*)()) &QConcentrationFlux::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QConcentration>(m, "QConcentration")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QConcentration::*)(const QConcentration& rhs) const) &QConcentration::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QConcentration::*)()) &QConcentration::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QConcentrationFlowRate>(m, "QConcentrationFlowRate")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QConcentrationFlowRate::*)(const QConcentrationFlowRate& rhs) const) &QConcentrationFlowRate::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QConcentrationFlowRate::*)()) &QConcentrationFlowRate::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QMolarFlux>(m, "QMolarFlux")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QMolarFlux::*)(const QMolarFlux& rhs) const) &QMolarFlux::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QMolarFlux::*)()) &QMolarFlux::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QMolarFlowRate>(m, "QMolarFlowRate")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QMolarFlowRate::*)(const QMolarFlowRate& rhs) const) &QMolarFlowRate::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QMolarFlowRate::*)()) &QMolarFlowRate::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QMassFlux>(m, "QMassFlux")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QMassFlux::*)(const QMassFlux& rhs) const) &QMassFlux::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QMassFlux::*)()) &QMassFlux::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QMassFlowRate>(m, "QMassFlowRate")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QMassFlowRate::*)(const QMassFlowRate& rhs) const) &QMassFlowRate::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QMassFlowRate::*)()) &QMassFlowRate::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QMass>(m, "QMass")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QMass::*)(const QMass& rhs) const) &QMass::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QMass::*)()) &QMass::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QPerArea>(m, "QPerArea")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QPerArea::*)(const QPerArea& rhs) const) &QPerArea::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QPerArea::*)()) &QPerArea::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QPerLength>(m, "QPerLength")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QPerLength::*)(const QPerLength& rhs) const) &QPerLength::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QPerLength::*)()) &QPerLength::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QVolume>(m, "QVolume")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QVolume::*)(const QVolume& rhs) const) &QVolume::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QVolume::*)()) &QVolume::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QArea>(m, "QArea")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QArea::*)(const QArea& rhs) const) &QArea::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QArea::*)()) &QArea::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QLength>(m, "QLength")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QLength::*)(const QLength& rhs) const) &QLength::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QLength::*)()) &QLength::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QRate>(m, "QRate")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QRate::*)(const QRate& rhs) const) &QRate::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QRate::*)()) &QRate::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QTime>(m, "QTime")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QTime::*)(const QTime& rhs) const) &QTime::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QTime::*)()) &QTime::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QAngle>(m, "QAngle")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QAngle::*)(const QAngle& rhs) const) &QAngle::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QAngle::*)()) &QAngle::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::class_<QDimensionless>(m, "QDimensionless")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double(QDimensionless::*)(const QDimensionless& rhs) const) &QDimensionless::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double(QDimensionless::*)()) &QDimensionless::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
        py::object py_metre_per_second = py::cast(unit::metre_per_second);
        m.attr("metre_per_second") = py_metre_per_second;
        py::object py_per_pascal = py::cast(unit::per_pascal);
        m.attr("per_pascal") = py_per_pascal;
        py::object py_metre_pow5_per_second_per_mole = py::cast(unit::metre_pow5_per_second_per_mole);
        m.attr("metre_pow5_per_second_per_mole") = py_metre_pow5_per_second_per_mole;
        py::object py_metre_squared_per_second = py::cast(unit::metre_squared_per_second);
        m.attr("metre_squared_per_second") = py_metre_squared_per_second;
        py::object py_pascal_second_per_metre_cubed = py::cast(unit::pascal_second_per_metre_cubed);
        m.attr("pascal_second_per_metre_cubed") = py_pascal_second_per_metre_cubed;
        py::object py_metre_cubed_per_second = py::cast(unit::metre_cubed_per_second);
        m.attr("metre_cubed_per_second") = py_metre_cubed_per_second;
        py::object py_poiseuille = py::cast(unit::poiseuille);
        m.attr("poiseuille") = py_poiseuille;
        py::object py_pascals = py::cast(unit::pascals);
        m.attr("pascals") = py_pascals;
        py::object py_newtons = py::cast(unit::newtons);
        m.attr("newtons") = py_newtons;
        py::object py_per_metre_cubed = py::cast(unit::per_metre_cubed);
        m.attr("per_metre_cubed") = py_per_metre_cubed;
        py::object py_mole_per_kg = py::cast(unit::mole_per_kg);
        m.attr("mole_per_kg") = py_mole_per_kg;
        py::object py_metre_cubed_per_mole_per_second = py::cast(unit::metre_cubed_per_mole_per_second);
        m.attr("metre_cubed_per_mole_per_second") = py_metre_cubed_per_mole_per_second;
        py::object py_mole_per_metre_pow4 = py::cast(unit::mole_per_metre_pow4);
        m.attr("mole_per_metre_pow4") = py_mole_per_metre_pow4;
        py::object py_mole_per_metre_pow5_per_second = py::cast(unit::mole_per_metre_pow5_per_second);
        m.attr("mole_per_metre_pow5_per_second") = py_mole_per_metre_pow5_per_second;
        py::object py_mole_per_metre_cubed_per_second = py::cast(unit::mole_per_metre_cubed_per_second);
        m.attr("mole_per_metre_cubed_per_second") = py_mole_per_metre_cubed_per_second;
        py::object py_mole_per_metre_cubed = py::cast(unit::mole_per_metre_cubed);
        m.attr("mole_per_metre_cubed") = py_mole_per_metre_cubed;
        py::object py_mole_per_metre_squared_per_second = py::cast(unit::mole_per_metre_squared_per_second);
        m.attr("mole_per_metre_squared_per_second") = py_mole_per_metre_squared_per_second;
        py::object py_mole_per_second = py::cast(unit::mole_per_second);
        m.attr("mole_per_second") = py_mole_per_second;
        py::object py_kg = py::cast(unit::kg);
        m.attr("kg") = py_kg;
        py::object py_per_metre_squared = py::cast(unit::per_metre_squared);
        m.attr("per_metre_squared") = py_per_metre_squared;
        py::object py_per_metre = py::cast(unit::per_metre);
        m.attr("per_metre") = py_per_metre;
        py::object py_metres_cubed = py::cast(unit::metres_cubed);
        m.attr("metres_cubed") = py_metres_cubed;
        py::object py_metres_squared = py::cast(unit::metres_squared);
        m.attr("metres_squared") = py_metres_squared;
        py::object py_metres = py::cast(unit::metres);
        m.attr("metres") = py_metres;
        py::object py_per_second = py::cast(unit::per_second);
        m.attr("per_second") = py_per_second;
        py::object py_seconds = py::cast(unit::seconds);
        m.attr("seconds") = py_seconds;
        py::object py_dimensionless = py::cast(unit::dimensionless);
        m.attr("dimensionless") = py_dimensionless;
}
