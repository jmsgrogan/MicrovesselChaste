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
#include "AbstractDiscreteContinuumPde.hpp"

#include "PythonObjectConverters.hpp"
#include "AbstractDiscreteContinuumPde3_3.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();
typedef AbstractDiscreteContinuumPde<3,3 > AbstractDiscreteContinuumPde3_3;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractDiscreteContinuumPde3_3_Overloads : public AbstractDiscreteContinuumPde3_3{
    public:
    using AbstractDiscreteContinuumPde3_3::AbstractDiscreteContinuumPde;
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumPde3_3,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumPde3_3_class(py::module &m){
py::class_<AbstractDiscreteContinuumPde3_3 , AbstractDiscreteContinuumPde3_3_Overloads , std::shared_ptr<AbstractDiscreteContinuumPde3_3 >   >(m, "AbstractDiscreteContinuumPde3_3")
        .def(py::init< >())
        .def(
            "AddDiscreteSource", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::std::shared_ptr<DiscreteSource<3> >)) &AbstractDiscreteContinuumPde3_3::AddDiscreteSource, 
            " " , py::arg("pDiscreteSource") )
        .def(
            "ComputeIsotropicDiffusionTerm", 
            (::QDiffusivity(AbstractDiscreteContinuumPde3_3::*)()) &AbstractDiscreteContinuumPde3_3::ComputeIsotropicDiffusionTerm, 
            " "  )
        .def(
            "GetDiscreteSources", 
            (::std::vector<std::shared_ptr<DiscreteSource<3> >, std::allocator<std::shared_ptr<DiscreteSource<3> > > >(AbstractDiscreteContinuumPde3_3::*)()) &AbstractDiscreteContinuumPde3_3::GetDiscreteSources, 
            " "  )
        .def(
            "SetIsotropicDiffusionConstant", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::QDiffusivity)) &AbstractDiscreteContinuumPde3_3::SetIsotropicDiffusionConstant, 
            " " , py::arg("diffusivity") )
        .def(
            "SetReferenceConcentration", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::QConcentration)) &AbstractDiscreteContinuumPde3_3::SetReferenceConcentration, 
            " " , py::arg("referenceConcentration") )
        .def(
            "SetReferenceLength", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::QLength)) &AbstractDiscreteContinuumPde3_3::SetReferenceLength, 
            " " , py::arg("referenceLength") )
        .def(
            "SetReferenceTime", 
            (void(AbstractDiscreteContinuumPde3_3::*)(::QTime)) &AbstractDiscreteContinuumPde3_3::SetReferenceTime, 
            " " , py::arg("referenceTime") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumPde3_3::*)()) &AbstractDiscreteContinuumPde3_3::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
