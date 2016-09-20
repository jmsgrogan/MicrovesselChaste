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

#ifndef DISCRETECONTINUUMMESH_HPP_
#define DISCRETECONTINUUMMESH_HPP_

#include <vector>
#include "SmartPointers.hpp"
#include "ChastePoint.hpp"
#include "TetrahedralMesh.hpp"
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include "Part.hpp"
#include "Cell.hpp"
#include "DimensionalChastePoint.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"

// Jonathan Shewchuk's triangle and Hang Si's tetgen.
//#define REAL double
//#define VOID void
//#include "triangle.h"
//#include "tetgen.h"
//#undef REAL
//#undef VOID

// Forward declaration
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
class DiscreteContinuumMeshGenerator;

/**
 * This is a TetrahedralMesh with some extra functions for point locating, output of node locations
 * and connectivity and attribute storage. It uses its own version of ImportFromTetgen with fewer template arguements. There is
 * scope for merging this versions with the one in TetrahedralMesh.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class DiscreteContinuumMesh : public TetrahedralMesh<ELEMENT_DIM, SPACE_DIM>
{
    /**
     * For access to ImportFromMesher
     */
    friend class DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM>;

    /**
     * Store element-wise region markers
     */
    std::vector<unsigned> mAttributes;

    /**
     * The reference length scale for the mesh
     */
    units::quantity<unit::length> mReferenceLength;

    /**
     * A vtk representation
     */
    vtkSmartPointer<vtkUnstructuredGrid> mpVtkMesh;

    /**
     * Is the vtk representation up to date
     */
    bool mVtkRepresentationUpToDate;

public:

    /**
     * Constructor
     */
    DiscreteContinuumMesh();

    /**
     * Destructor
     */
    ~DiscreteContinuumMesh();

    /**
     *  Factory constructor method
     * @return a shared pointer to a new mesh
     */
    static boost::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > Create();

    /**
     * Return the element connectivity
     */
    std::vector<std::vector<unsigned> > GetConnectivity();

    /**
     * Return the node locations
     */
    std::vector<std::vector<double> > GetNodeLocations();

    /**
     * Return the node locations
     */
    std::vector<DimensionalChastePoint<SPACE_DIM> > GetNodeLocationsAsPoints();

    /**
     * Return the element-wise region markers
     */
    std::vector<unsigned> GetElementRegionMarkers();

    /**
     * Return the mesh as a vtk unstructured grid
     */
    vtkSmartPointer<vtkUnstructuredGrid> GetAsVtkUnstructuredGrid();

    /**
     * Set element attributes
     */
    void SetAttributes(std::vector<unsigned> attributes);

    /**
     * This is the same as the TetrahedralMesh implementation of ImportFromMesher but avoids some templating
     */
    void ImportDiscreteContinuumMeshFromTetgen(tetgen::tetgenio& mesherOutput, unsigned numberOfElements, int *elementList,
                          unsigned numberOfFaces, int *faceList, int *edgeMarkerList);

};

#endif /* DISCRETECONTINUUMMESH_HPP_*/
