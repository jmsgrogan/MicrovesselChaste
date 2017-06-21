// This file has been generated by Py++.

/*

Copyright (c) 2005-2017, University of Oxford.
All rights reserved.

University of Oxford means the Chancellor, Masters and Scholars of the
University of Oxford, having an administrative office at Wellington
Square, Oxford OX1 2JD, UK.

This file is part of Chaste.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of the University of Oxford nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "boost/python.hpp"

#include "indexing_suite/value_traits.hpp"

#include "indexing_suite/container_suite.hpp"

#include "indexing_suite/vector.hpp"

#include "indexing_suite/map.hpp"

#include "wrapper_header_collection.hpp"

#include "BoundaryExtractor.pypp.hpp"

#include "Facet2.pypp.hpp"

#include "Facet3.pypp.hpp"

#include "GeometryFormat.pypp.hpp"

#include "GeometryWriter.pypp.hpp"

#include "MapStringDouble.pypp.hpp"

#include "MappableGridGenerator2.pypp.hpp"

#include "MappableGridGenerator3.pypp.hpp"

#include "NetworkToSurface2.pypp.hpp"

#include "NetworkToSurface3.pypp.hpp"

#include "Part2.pypp.hpp"

#include "Part3.pypp.hpp"

#include "Polygon2.pypp.hpp"

#include "Polygon3.pypp.hpp"

#include "SurfaceCleaner.pypp.hpp"

#include "VectorBool.pypp.hpp"

#include "VectorDimensionalChastePoint2.pypp.hpp"

#include "VectorDimensionalChastePoint3.pypp.hpp"

#include "VectorLengthQuantity.pypp.hpp"

#include "VectorMapStringDouble.pypp.hpp"

#include "VectorPairDimensionalChastePoint3Unsigned.pypp.hpp"

#include "VectorSharedPtrDimensionalChastePoint2.pypp.hpp"

#include "VectorSharedPtrDimensionalChastePoint3.pypp.hpp"

#include "VectorSharedPtrFacet2.pypp.hpp"

#include "VectorSharedPtrFacet3.pypp.hpp"

#include "VectorSharedPtrPolygon2.pypp.hpp"

#include "VectorSharedPtrPolygon3.pypp.hpp"

#include "VectorUnsigned.pypp.hpp"

#include "VesselSurfaceGenerator2.pypp.hpp"

#include "VesselSurfaceGenerator3.pypp.hpp"

#include "__type.pypp.hpp"

#include "map_less__unsigned_int_comma__std_scope_string__greater_.pypp.hpp"

#include "vector_less__std_scope_pair_less_std_scope_pair_less_unsigned_int_comma__unsigned_int_greater__comma__unsigned_int_greater___greater_.pypp.hpp"

#include "vector_less__std_scope_vector_less__boost_scope_shared_ptr_less_Polygon_less_2_greater___greater___greater___greater_.pypp.hpp"

#include "vector_less__std_scope_vector_less__boost_scope_shared_ptr_less_Polygon_less_3_greater___greater___greater___greater_.pypp.hpp"

namespace bp = boost::python;

BOOST_PYTHON_MODULE(_chaste_project_MicrovesselChaste_geometry){
    register_VectorUnsigned_class();

    register_vector_less__std_scope_vector_less__boost_scope_shared_ptr_less_Polygon_less_3_greater___greater___greater___greater__class();

    register_vector_less__std_scope_vector_less__boost_scope_shared_ptr_less_Polygon_less_2_greater___greater___greater___greater__class();

    register_vector_less__std_scope_pair_less_std_scope_pair_less_unsigned_int_comma__unsigned_int_greater__comma__unsigned_int_greater___greater__class();

    register_VectorPairDimensionalChastePoint3Unsigned_class();

    register___type_class();

    register_VectorMapStringDouble_class();

    register_VectorLengthQuantity_class();

    register_VectorSharedPtrPolygon3_class();

    register_VectorSharedPtrPolygon2_class();

    register_VectorSharedPtrFacet3_class();

    register_VectorSharedPtrFacet2_class();

    register_VectorSharedPtrDimensionalChastePoint3_class();

    register_VectorSharedPtrDimensionalChastePoint2_class();

    register_VectorBool_class();

    register_VectorDimensionalChastePoint3_class();

    register_VectorDimensionalChastePoint2_class();

    register_map_less__unsigned_int_comma__std_scope_string__greater__class();

    register_MapStringDouble_class();

    register_BoundaryExtractor_class();

    register_Facet2_class();

    register_Facet3_class();

    register_GeometryFormat_class();

    register_GeometryWriter_class();

    register_MappableGridGenerator2_class();

    register_MappableGridGenerator3_class();

    register_NetworkToSurface2_class();

    register_NetworkToSurface3_class();

    register_Part2_class();

    register_Part3_class();

    register_Polygon2_class();

    register_Polygon3_class();

    register_SurfaceCleaner_class();

    register_VesselSurfaceGenerator2_class();

    register_VesselSurfaceGenerator3_class();
}

