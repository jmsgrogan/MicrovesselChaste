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

#include "RegularGrid.hpp"
#include "RegularGridWriter.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "MeshReader.hpp"
#include "MultiFormatMeshWriter.hpp"
#include "DimensionalChastePoint.hpp"

inline int Instantiation()
{
    return
            sizeof(RegularGrid<3>) +
            sizeof(RegularGrid<2>) +
            sizeof(DiscreteContinuumMesh<3, 3>) +
            sizeof(DiscreteContinuumMesh<2, 2>) +
            sizeof(DiscreteContinuumMeshGenerator<3,3>)+
            sizeof(DiscreteContinuumMeshGenerator<2,2>)+
            sizeof(MultiFormatMeshWriter<3>)+
            sizeof(MultiFormatMeshWriter<2>)+
            sizeof(DimensionalChastePoint<3>)+
            sizeof(DimensionalChastePoint<2>);
}
