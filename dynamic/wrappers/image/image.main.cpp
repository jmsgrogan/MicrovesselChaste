#include <pybind11/pybind11.h>
#include "ImageReader.cppwg.hpp"
#include "ImageToMesh2.cppwg.hpp"
#include "ImageToMesh3.cppwg.hpp"
#include "ImageToSurface.cppwg.hpp"
#include "NetworkToImage2.cppwg.hpp"
#include "NetworkToImage3.cppwg.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_chaste_project_MicrovesselChaste_image, m)
{
    register_ImageReader_class(m);
    register_ImageToMesh2_class(m);
    register_ImageToMesh3_class(m);
    register_ImageToSurface_class(m);
    register_NetworkToImage2_class(m);
    register_NetworkToImage3_class(m);
}
