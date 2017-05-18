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

#include "indexing_suite/set.hpp"

#include "indexing_suite/map.hpp"

#include "wrapper_header_collection.hpp"

#include "AbstractDiscreteContinuumGrid2_2.pypp.hpp"

#include "AbstractDiscreteContinuumGrid3_3.pypp.hpp"

#include "AbstractMesh2_2.pypp.hpp"

#include "AbstractMesh3_3.pypp.hpp"

#include "AbstractTetrahedralMesh2_2.pypp.hpp"

#include "AbstractTetrahedralMesh3_3.pypp.hpp"

#include "DimensionalChastePoint2.pypp.hpp"

#include "DimensionalChastePoint3.pypp.hpp"

#include "DiscreteContinuumMesh2_2.pypp.hpp"

#include "DiscreteContinuumMesh3_3.pypp.hpp"

#include "DiscreteContinuumMeshGenerator2_2.pypp.hpp"

#include "DiscreteContinuumMeshGenerator3_3.pypp.hpp"

#include "GridCalculator2.pypp.hpp"

#include "GridCalculator3.pypp.hpp"

#include "MapStringDouble.pypp.hpp"

#include "MeshReader.pypp.hpp"

#include "MultiFormatMeshWriter2.pypp.hpp"

#include "MultiFormatMeshWriter3.pypp.hpp"

#include "RegularGrid2.pypp.hpp"

#include "RegularGrid3.pypp.hpp"

#include "RegularGridWriter.pypp.hpp"

#include "SetUnsigned.pypp.hpp"

#include "TetrahedralMesh2_2.pypp.hpp"

#include "TetrahedralMesh3_3.pypp.hpp"

#include "VectorDimensionalChastePoint2.pypp.hpp"

#include "VectorDimensionalChastePoint3.pypp.hpp"

#include "VectorDouble.pypp.hpp"

#include "VectorUnsigned.pypp.hpp"

#include "VectorVectorSharedPtrCell.pypp.hpp"

#include "VectorVectorSharedPtrVesselNode2.pypp.hpp"

#include "VectorVectorSharedPtrVesselNode3.pypp.hpp"

#include "VectorVectorSharedPtrVesselSegment2.pypp.hpp"

#include "VectorVectorSharedPtrVesselSegment3.pypp.hpp"

#include "VectorVectorUnsigned.pypp.hpp"

#include "__type.pypp.hpp"

#include "__type.pypp.hpp"

namespace bp = boost::python;

BOOST_PYTHON_MODULE(_chaste_project_MicrovesselChaste_mesh){
    register_VectorUnsigned_class();

    register_VectorVectorUnsigned_class();

    register_VectorVectorSharedPtrVesselSegment3_class();

    register_VectorVectorSharedPtrVesselSegment2_class();

    register_VectorVectorSharedPtrVesselNode3_class();

    register_VectorVectorSharedPtrVesselNode2_class();

    register_VectorVectorSharedPtrCell_class();

    register_VectorDouble_class();

    register___type_class();

    register___type_class();

    register_VectorDimensionalChastePoint3_class();

    register_VectorDimensionalChastePoint2_class();

    register_SetUnsigned_class();

    register_MapStringDouble_class();

    register_AbstractDiscreteContinuumGrid2_2_class();

    register_AbstractDiscreteContinuumGrid3_3_class();

    register_AbstractMesh2_2_class();

    register_AbstractMesh3_3_class();

    register_AbstractTetrahedralMesh2_2_class();

    register_AbstractTetrahedralMesh3_3_class();

    register_DimensionalChastePoint2_class();

    register_DimensionalChastePoint3_class();

    register_TetrahedralMesh2_2_class();

    register_DiscreteContinuumMesh2_2_class();

    register_TetrahedralMesh3_3_class();

    register_DiscreteContinuumMesh3_3_class();

    register_DiscreteContinuumMeshGenerator2_2_class();

    register_DiscreteContinuumMeshGenerator3_3_class();

    register_GridCalculator2_class();

    register_GridCalculator3_class();

    register_MeshReader_class();

    register_MultiFormatMeshWriter2_class();

    register_MultiFormatMeshWriter3_class();

    register_RegularGrid2_class();

    register_RegularGrid3_class();

    register_RegularGridWriter_class();
}
