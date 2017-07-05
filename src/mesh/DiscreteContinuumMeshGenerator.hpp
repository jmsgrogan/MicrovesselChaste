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

#ifndef DISCRETECONTINUUMMESHGENERATOR_HPP_
#define DISCRETECONTINUUMMESHGENERATOR_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include "SmartPointers.hpp"
#include "ChastePoint.hpp"
#include "TetrahedralMesh.hpp"
#include "Part.hpp"
#include "Cell.hpp"
#include "DimensionalChastePoint.hpp"

// Jonathan Shewchuk's triangle and Hang Si's tetgen.
#define REAL double
#define VOID void
#include "triangle.h"
#include "tetgen.h"
#undef REAL
#undef VOID

struct triangulateio;

// Forward declaration
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
class DiscreteContinuumMesh;

/**
 * This class is for generating 2d and 3d finite element meshes using triangle and tetgen.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class DiscreteContinuumMeshGenerator
{
    /**
     * Max area argument used in mesh generation.
     */
    units::quantity<unit::volume> mMaxElementArea;

    /**
     * The mesh
     */
    std::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > mpMesh;

    /**
     * A part to be meshed. This may not be the final mesh geometry if extra vtk polydata is included.
     */
    std::shared_ptr<Part<SPACE_DIM> > mpDomain;

    /**
     * A vtk surface to be meshed
     */
    vtkSmartPointer<vtkPolyData> mpVtkDomain;

    /**
     * A path to an stl surface to be meshed
     */
    std::string mStlFilePath;

    /**
     * A collection of hole location markers
     */
    std::vector<DimensionalChastePoint<SPACE_DIM> > mHoles;

    /**
     * A collection of region markers
     */
    std::vector<DimensionalChastePoint<SPACE_DIM> > mRegions;

    /**
     * Store element-wise region markers
     */
    std::vector<unsigned> mAttributes;

    /**
     * The reference length scale for the mesh
     */
    QLength mReferenceLength;

public:

    /**
     * Constructor
     */
    DiscreteContinuumMeshGenerator();

    /**
     * Destructor
     */
    ~DiscreteContinuumMeshGenerator();

    /**
     * Factory constructor method
     * @return a shared pointer to a new mesh generator
     */
    static std::shared_ptr<DiscreteContinuumMeshGenerator<ELEMENT_DIM, SPACE_DIM> > Create();

    /**
     * Return the mesh
     * @return the mesh
     */
    std::shared_ptr<DiscreteContinuumMesh<ELEMENT_DIM, SPACE_DIM> > GetMesh();

    /**
     * Set the domain for meshing
     * @param pDomain the domain for meshing
     */
    void SetDomain(std::shared_ptr<Part<SPACE_DIM> > pDomain);

    /**
     * Set the domain for meshing
     * @param pDomain the domain for meshing
     */
    void SetDomain(vtkSmartPointer<vtkPolyData> pDomain);

    /**
     * Set the domain for meshing
     * @param rPathToStl the path to the stl for meshing
     */
    void SetDomain(const std::string& rPathToStl);

    /**
     * Set the max element area
     * @param maxElementArea the max element area
     */
    void SetMaxElementArea(units::quantity<unit::volume> maxElementArea);

    /**
    * Set the hole locations
    * @param holes hole locations
    */
    void SetHoles(std::vector<DimensionalChastePoint<SPACE_DIM> > holes);

    /**
    * Set the region marker locations
    * @param regionMarkers region marker locations
    */
    void SetRegionMarkers(std::vector<DimensionalChastePoint<SPACE_DIM> > regionMarkers);

    /**
     * Do the meshing
     */
    void Update();

private:

    /**
     * Use triangle for 2-D meshing
     */
    void Mesh2d();

    /**
     * Use tetgen for 3-D meshing
     */
    void Mesh3d();

    /**
     * Use tetgen for 3-D meshing of an stl
     */
    void MeshStl3d();

    /**
     * This is the same as the TetrahedralMesh implementation of InitialiseTriangulateIo but avoids some templating
     * @param mesherIo the mesher input
     */
    void InitialiseTriangulateIo(triangulateio& mesherIo);

    /**
     * This is the same as the TetrahedralMesh implementation of InitialiseTriangulateIo but avoids some templating
     * @param mesherIo the mesher input
     */
    void FreeTriangulateIo(triangulateio& mesherIo);
};

#endif /* DISCRETECONTINUUMMESHGENERATOR_HPP_*/
