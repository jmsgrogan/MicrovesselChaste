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

#ifndef MESHREADER_HPP_
#define MESHREADER_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>
#include "SmartPointers.hpp"

/**
 * Read a vtk unstructured mesh from file. This is a simple wrapper over the vtk unstructured grid reader.
 */
class MeshReader
{
    /**
     *  The mesh in vtk format
     */
    vtkSmartPointer<vtkUnstructuredGrid> mpVtkMesh;

    /**
     *  Path to the mesh file
     */
    std::string mFilepath;

public:

    /**
     * Constructor
     */
    MeshReader();

    /**
     * Destructor
     */
    ~MeshReader();

    /**
     *  Factory constructor method
     * @return a shared pointer to a new instance
     */
    static std::shared_ptr<MeshReader> Create();

    /**
     * Get the mesh in vtu format
     * @return the mesh in vtu format
     */
    vtkSmartPointer<vtkUnstructuredGrid> GetMesh();

    /**
     * Set the filename for the readers and writers
     * @param rFilename the file name
     */
    void SetFileName(const std::string& rFilename);

    /**
     * Do the read operation
     */
    void Read();

};

#endif /* MESHREADER_HPP_*/
