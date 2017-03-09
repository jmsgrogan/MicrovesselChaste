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

/*

Copyright (c) 2005-2016, University of Oxford.
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

#ifndef MESH_HEADERS_HPP
#define MESH_HEADERS_HPP

#include "AbstractDiscreteContinuumGrid.hpp"
#include "GridCalculator.hpp"
#include "RegularGrid.hpp"
#include "RegularGridWriter.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "MeshReader.hpp"
#include "MultiFormatMeshWriter.hpp"
#include "DimensionalChastePoint.hpp"

namespace pyplusplus{
namespace aliases{
typedef AbstractDiscreteContinuumGrid<3> AbstractDiscreteContinuumGrid3;
typedef AbstractDiscreteContinuumGrid<2> AbstractDiscreteContinuumGrid2;
typedef GridCalculator<3> GridCalculator3;
typedef GridCalculator<2> GridCalculator2;
typedef RegularGrid<3> RegularGrid3;
typedef RegularGrid<2> RegularGrid2;
typedef DiscreteContinuumMesh<3, 3> DiscreteContinuumMesh3_3;
typedef DiscreteContinuumMesh<2, 2> DiscreteContinuumMesh2_2;
typedef DiscreteContinuumMeshGenerator<3, 3> DiscreteContinuumMeshGenerator3_3;
typedef DiscreteContinuumMeshGenerator<2, 2> DiscreteContinuumMeshGenerator2_2;
typedef MultiFormatMeshWriter<3> MultiFormatMeshWriter3;
typedef MultiFormatMeshWriter<2> MultiFormatMeshWriter2;
typedef DimensionalChastePoint<3> DimensionalChastePoint3;
typedef DimensionalChastePoint<2> DimensionalChastePoint2;
typedef std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > > VecDimensionalChastePoint2;
typedef std::vector<DimensionalChastePoint<3>, std::allocator<DimensionalChastePoint<3> > > VecDimensionalChastePoint3;

// These are defined in the PyChaste preload module. They are included here to avoid duplicate registration warnings.
typedef std::vector<unsigned int> VecUnsignedInt_MeshModule;
typedef std::vector<double> VecDouble_MeshModule;
}
}//pyplusplus::aliases

template class AbstractDiscreteContinuumGrid<3>;
template class AbstractDiscreteContinuumGrid<2>;
template class GridCalculator<3>;
template class GridCalculator<2>;
template class RegularGrid<3>;
template class RegularGrid<2>;
template class DiscreteContinuumMesh<3, 3>;
template class DiscreteContinuumMesh<2, 2>;
template class DiscreteContinuumMeshGenerator<3, 3>;
template class DiscreteContinuumMeshGenerator<2, 2>;
template class MultiFormatMeshWriter<3>;
template class MultiFormatMeshWriter<2>;
template class DimensionalChastePoint<3>;
template class DimensionalChastePoint<2>;

#endif
