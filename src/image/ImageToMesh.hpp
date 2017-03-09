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



#ifndef ImageToMesh_HPP_
#define ImageToMesh_HPP_
#include "SmartPointers.hpp"
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkImageData.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include "DiscreteContinuumMesh.hpp"
#include "Part.hpp"

/**
* Do finite element meshing of a region in an image
*/
template<unsigned DIM>
class ImageToMesh
{
    /**
     *  The image
     */
    vtkSmartPointer<vtkImageData> mpImage;

    /**
     *  The target element size
     */
    units::quantity<unit::volume> mElementSize;

    /**
     *  The mesh
     */
    boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > mMesh;

    /**
     * Tissue domain (optional)
     */
    boost::shared_ptr<Part<DIM> > mpDomain;

public:

    /**
     * Constructor
     */
    ImageToMesh();

    /**
     * Destructor
     */
    ~ImageToMesh();

    /**
     * Factory constructor method
     * @return a pointer to the converter
     */
    static boost::shared_ptr<ImageToMesh<DIM> > Create();

    /**
     * Get the mesh
     * @return the finite element mesh
     */
    boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > GetMesh();

    /**
     * Set the element size
     * @param elementSize the element volume
     */
    void SetElementSize(units::quantity<unit::volume> elementSize);

    /**
     * Set the image to be meshed
     * @param pImage the input image
     */
    void SetInput(vtkSmartPointer<vtkImageData> pImage);

    /**
     * Set the image to be meshed using a raw pointer, needed for Python wrapping
     * @param pImage a raw pointer to the input image
     */
    void SetInputRaw(vtkImageData* pImage);

    /**
     * Add a tissue domain to the mesh
     * @param pTissueDomain a tissue domain for meshing
     */
    void SetTissueDomain(boost::shared_ptr<Part<DIM> > pTissueDomain);

    /**
     * Do the meshing
     */
    void Update();

};

#endif /*ImageToMesh_HPP_*/
