#include <pybind11/pybind11.h>
#include "BaseUnits.cppwg.hpp"
#include "ParameterCollection.cppwg.hpp"
#include "Owen11Parameters.cppwg.hpp"
#include "Connor17Parameters.cppwg.hpp"
#include "Secomb04Parameters.cppwg.hpp"
#include "GenericParameters.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_utility, m)
{
    register_BaseUnits_class(m);
    register_ParameterCollection_class(m);
    register_Owen11Parameters_class(m);
    register_Connor17Parameters_class(m);
    register_Secomb04Parameters_class(m);
    register_GenericParameters_class(m);
}
