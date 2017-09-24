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
#include "AbstractDiscreteContinuumPde2_2.cppwg.hpp"

namespace py = pybind11;
PYBIND11_CVECTOR_TYPECASTER2();
PYBIND11_CVECTOR_TYPECASTER3();   
typedef AbstractDiscreteContinuumPde<2,2 > AbstractDiscreteContinuumPde2_2;
PYBIND11_DECLARE_HOLDER_TYPE(T, std::shared_ptr<T>);

class AbstractDiscreteContinuumPde2_2_Overloads : public AbstractDiscreteContinuumPde2_2{
    public:
    using AbstractDiscreteContinuumPde2_2::AbstractDiscreteContinuumPde;
    void UpdateDiscreteSourceStrengths() override {
        PYBIND11_OVERLOAD(
            void,
            AbstractDiscreteContinuumPde2_2,
            UpdateDiscreteSourceStrengths,
            );
    }

};
void register_AbstractDiscreteContinuumPde2_2_class(py::module &m){
py::class_<AbstractDiscreteContinuumPde2_2 , AbstractDiscreteContinuumPde2_2_Overloads , std::shared_ptr<AbstractDiscreteContinuumPde2_2 >   >(m, "AbstractDiscreteContinuumPde2_2")
        .def(py::init< >())
        .def(
            "AddDiscreteSource", 
            (void(AbstractDiscreteContinuumPde2_2::*)(::std::shared_ptr<DiscreteSource<2> >)) &AbstractDiscreteContinuumPde2_2::AddDiscreteSource, 
            " " , py::arg("pDiscreteSource") )
        .def(
            "ComputeIsotropicDiffusionTerm", 
            (::QDiffusivity(AbstractDiscreteContinuumPde2_2::*)()) &AbstractDiscreteContinuumPde2_2::ComputeIsotropicDiffusionTerm, 
            " "  )
        .def(
            "GetDiscreteSources", 
            (::std::vector<std::shared_ptr<DiscreteSource<2> >, std::allocator<std::shared_ptr<DiscreteSource<2> > > >(AbstractDiscreteContinuumPde2_2::*)()) &AbstractDiscreteContinuumPde2_2::GetDiscreteSources, 
            " "  )
        .def(
            "SetIsotropicDiffusionConstant", 
            (void(AbstractDiscreteContinuumPde2_2::*)(::QDiffusivity)) &AbstractDiscreteContinuumPde2_2::SetIsotropicDiffusionConstant, 
            " " , py::arg("diffusivity") )
        .def(
            "SetReferenceConcentration", 
            (void(AbstractDiscreteContinuumPde2_2::*)(::QConcentration)) &AbstractDiscreteContinuumPde2_2::SetReferenceConcentration, 
            " " , py::arg("referenceConcentration") )
        .def(
            "SetReferenceLength", 
            (void(AbstractDiscreteContinuumPde2_2::*)(::QLength)) &AbstractDiscreteContinuumPde2_2::SetReferenceLength, 
            " " , py::arg("referenceLength") )
        .def(
            "SetReferenceTime", 
            (void(AbstractDiscreteContinuumPde2_2::*)(::QTime)) &AbstractDiscreteContinuumPde2_2::SetReferenceTime, 
            " " , py::arg("referenceTime") )
        .def(
            "UpdateDiscreteSourceStrengths", 
            (void(AbstractDiscreteContinuumPde2_2::*)()) &AbstractDiscreteContinuumPde2_2::UpdateDiscreteSourceStrengths, 
            " "  )
    ;
}
