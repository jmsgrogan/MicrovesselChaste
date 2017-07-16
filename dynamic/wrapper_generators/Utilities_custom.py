
import cppwg.templates.custom


class Utilities_custom(cppwg.templates.custom.Custom):

    def __init__(self):
        pass
    
    def get_module_pre_code(self):
        
        code = '#include "UnitCollection.hpp"\n'
        code += '#include "VectorUnitCollection.hpp"\n'
        code += '#include <pybind11/operators.h>\n'
        code += '#include "UblasIncludes.hpp"\n'
        code += '#include "PythonObjectConverters.hpp"\n'
        code += 'PYBIND11_CVECTOR_TYPECASTER3();\n'
        code += 'PYBIND11_CVECTOR_TYPECASTER2();\n'
        return code
    
    def get_module_code(self):
        
        code = ""
        vector_quantity_names = ["VecQLength"]

        for eachQuantity in vector_quantity_names:
            replacements = {'class': eachQuantity}
            internal_code = """\
        py::class_<{class}<2> >(m, "{class}2")
        .def(py::init< >())
        .def(py::init<c_vector<double, 2>, QLength >())
        .def(py::init<double >())
        .def("Convert", 
            (c_vector<double, 2>({class}<2>::*)(const QLength& rhs) const) &{class}<2>::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (c_vector<double, 2>({class}<2>::*)()) &{class}<2>::GetValue, 
            " " )
    ;
        py::class_<{class}<3> >(m, "{class}3")
        .def(py::init< >())
        .def(py::init<c_vector<double, 3>, QLength >())
        .def(py::init<double >())
        .def("Convert", 
            (c_vector<double, 3>({class}<3>::*)(const QLength& rhs) const) &{class}<3>::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (c_vector<double, 3>({class}<3>::*)()) &{class}<3>::GetValue, 
            " " )
    ;
"""        
            code += internal_code.format(**replacements)
        
        quantity_names = ["QVolumetricSolubility",
                  "QSolubility",
                  "QDiffusivityPerConcentration",
                  "QDiffusivity",
                  "QFlowRate",
                  "QFlowImpedance",
                  "QDynamicViscosity",
                  "QPressure",
                  "QForce",
                  "QVelocity",
                  "QNumberDensity",
                  "QMolarMass",
                  "QRatePerConcentration",
                  "QConcentrationGradient",
                  "QConcentrationFlux",
                  "QConcentration",
                  "QConcentrationFlowRate",
                  "QMolarFlux",
                  "QMolarFlowRate",
                  "QMassFlux",
                  "QMassFlowRate",
                  "QMass",
                  "QPerArea",
                  "QPerLength",
                  "QVolume",
                  "QArea",
                  "QLength",
                  "QRate",
                  "QTime",
                  "QAngle",
                  "QDimensionless"
                  ]
        
        for eachQuantity in quantity_names:
            replacements = {'class': eachQuantity}
            internal_code = """\
        py::class_<{class}>(m, "{class}")
        .def(py::init< >())
        .def(py::init<double >())
        .def("Convert", 
            (double({class}::*)(const {class}& rhs) const) &{class}::Convert, 
            " " , py::arg("rhs"))
        .def("GetValue", 
            (double({class}::*)()) &{class}::GetValue, 
            " " )
        .def(py::self + py::self)
        .def(py::self * py::self)
        .def(py::self / py::self)
        .def(py::self - py::self)
        .def(float() * py::self)
        .def(py::self / float())
    ;
"""
            code += internal_code.format(**replacements)
            
            units = ["metre_per_second", 
                     "per_pascal",
                     "metre_pow5_per_second_per_mole",
                     "metre_squared_per_second",
                     "pascal_second_per_metre_cubed",
                     "metre_cubed_per_second",
                     "poiseuille",
                     "pascals",
                     "newtons",
                     "per_metre_cubed",
                     "mole_per_kg",
                     "metre_cubed_per_mole_per_second",
                     "mole_per_metre_pow4",
                     "mole_per_metre_pow5_per_second",
                     "mole_per_metre_cubed_per_second",
                     "mole_per_metre_cubed",
                     "mole_per_metre_squared_per_second",
                     "mole_per_second",
                     "kg",
                     "per_metre_squared",
                     "per_metre",
                     "metres_cubed",
                     "metres_squared",
                     "metres",
                     "per_second",
                     "seconds", 
                     "dimensionless",       
                     ]
        
        for eachUnit in units:
            replacements = {'unit': eachUnit}
            internal_code = """\
        py::object py_{unit} = py::cast(unit::{unit});
        m.attr("{unit}") = py_{unit};
"""
            code += internal_code.format(**replacements)         
        return code