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

#ifndef IMAGEREADER_HPP_
#define IMAGEREADER_HPP_

#include "SmartPointers.hpp"
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkImageData.h>
#include <vtkSmartPointer.h>

/**
 *  This class reads images in VTK compatible formats.
 *  The image can be optionally resized, which often needs to be the first operation in the workflow for memory reasons.
 */
class ImageReader
{

private:

    /**
     *  The image in vtk format
     */
    vtkSmartPointer<vtkImageData> mpVtkImage;

    /**
     *  Path to the image
     */
    std::string mFilepath;

    /**
     *  The fraction of pixels to retain in X after reading
     */
    double mResizeX;

    /**
     *  The fraction of pixels to retain in Y after reading
     */
    double mResizeY;

    /**
     *  The fraction of pixels to retain in Z after reading
     */
    double mResizeZ;

public:

    /**
     * Constructor
     */
    ImageReader();

    /**
     * Destructor
     */
    ~ImageReader();

    /**
     * Factory constructor method
     * @return a shared pointer to a instance of this class
     */
    static boost::shared_ptr<ImageReader> Create();

    /**
     * Get the image in vti format
     * @return the image in vti format
     */
    vtkSmartPointer<vtkImageData> GetImage();

    /**
     * Set the filename for the readers and writers
     * @param rFilename the file name
     */
    void SetFilename(const std::string& rFilename);

    /**
     * Set the resize factors for the image
     * @param factorX the fraction of pixels to retain in x
     * @param factorY the fraction of pixels to retain in y
     * @param factorZ the fraction of pixels to retain in z
     */
    void SetImageResizeFactors(double factorX, double factorY=1.0, double factorZ=1.0);

    /**
     * Do the read operation
     */
    void Read();

};

#endif /*ImageReader_HPP_*/
