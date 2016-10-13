/*

 Copyright (c) 2005-2015, University of Oxford.
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

#ifndef MultiFormatMeshWriter_HPP_
#define MultiFormatMeshWriter_HPP_

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>
#include "SmartPointers.hpp"
#include "DiscreteContinuumMesh.hpp"

/**
 * Helper struct for specifying the output mesh format
 */
struct MeshFormat
{
    enum Value
    {
        VTU, DOLFIN, STL
    };
};

/**
 *  Write meshes represented as vtk unstructured grids or DiscreteContinuumMesh to file.
 *  Supports output in VTU, STL (ascii) and dolfin formats. This differs from the VtkMeshWriter in that it allows
 *  the full path to an output directory to be set and has multiple output types. It does not write parallel
 *  vtk files.
 */
template<unsigned DIM>
class MultiFormatMeshWriter
{

private:

    /**
     *  The mesh to be written in vtk format
     */
    vtkSmartPointer<vtkUnstructuredGrid> mpVtkMesh;


    /**
     *  The mesh to be written in DiscreteContinuumMesh format
     */
    boost::shared_ptr<DiscreteContinuumMesh<DIM> > mpMesh;

    /**
     *  The output path
     */
    std::string mFilepath;

    /**
     * The output mesh format
     */
    MeshFormat::Value mOutputFormat;

public:

    /**
     * Constructor
     */
    MultiFormatMeshWriter();

    /**
     * Destructor
     */
    ~MultiFormatMeshWriter();

    /**
     * Factory constructor method
     * @return a shared pointer to a instance of this class
     */
    static boost::shared_ptr<MultiFormatMeshWriter> Create();

    /**
     * Set the filename for the writer without extension
     * @param rFilename the file name without extension
     */
    void SetFilename(const std::string& rFilename);

    /**
     * Set the mesh in vtu format
     * @param pMesh the mesh to write out
     */
    void SetMesh(vtkSmartPointer<vtkUnstructuredGrid> pMesh);

    /**
     * Set the mesh in DiscreteContinuum mesh format
     * @param pMesh the mesh to write out
     */
    void SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh);

    /**
     * Set the format for writer output
     * @param outputFormat the output format for the mesh writer
     */
    void SetOutputFormat(MeshFormat::Value outputFormat);

    /**
     * Write the mesh
     */
    void Write();
};

#endif /*MultiFormatMeshWriter_HPP_*/
