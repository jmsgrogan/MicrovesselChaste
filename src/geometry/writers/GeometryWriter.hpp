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
 * Redistributions in binary form must reproduce the abovea copyright notice,
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

#ifndef GEOMETRYWRITER_HPP_
#define GEOMETRYWRITER_HPP_

#include <string>
#define _BACKWARD_BACKWARD_WARNING_H 1
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

/**
 * Helper struct for specifying the output geometry format
 */
struct GeometryFormat
{
    /**
     * Can be vtk polydata (VTP) or STL format
     */
    enum Value
    {
        VTP, STL
    };
};

/**
 * This class writes geometry information stored as vtk polydata to file in VTP or ascci STL format.
 * For the latter it is assume that the geometry is triangulated.
 */
class GeometryWriter
{

private:

    /**
     * The geometry to be written
     */
    vtkSmartPointer<vtkPolyData> mpInputGeometry;

    /**
     * The output file name
     */
    std::string mFilename;

    /**
     * The output format
     */
    GeometryFormat::Value mFormat;

public:

    /**
     * Constructor
     */
    GeometryWriter();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a shared pointer to the class
     */
    static boost::shared_ptr<GeometryWriter> Create();

    /**
     * Destructor
     */
    ~GeometryWriter();

    /**
     * Set the geometry to be written in vtk format
     * @param pSurface the geometry to be written
     */
    void SetInput(vtkSmartPointer<vtkPolyData> pSurface);

    /**
     * Set the output filename, without extension
     * @param rFileName the full output path without extension
     */
    void SetFileName(const std::string& rFileName);

    /**
     * Set the output format, VTP or STL
     * @param format the output format, VTP or STL
     */
    void SetOutputFormat(GeometryFormat::Value format);

    /**
     * Do the write
     */
    void Write();

};

#endif /* GeometryWriter_HPP_ */
