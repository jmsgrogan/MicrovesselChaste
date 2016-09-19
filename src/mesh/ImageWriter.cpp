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

#include "Exception.hpp"
#include <boost/filesystem.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkXMLImageDataWriter.h>
#include <vtkVersion.h>
#include "PetscTools.hpp"
#include "ImageWriter.hpp"

ImageWriter::ImageWriter()
    : mpVtkImage(vtkSmartPointer<vtkImageData>::New()),
      mFilepath()
{

}

boost::shared_ptr<ImageWriter> ImageWriter::Create()
{
    MAKE_PTR(ImageWriter, pSelf);
    return pSelf;
}

ImageWriter::~ImageWriter()
{

}

void ImageWriter::SetImage(vtkSmartPointer<vtkImageData> pImage)
{
    mpVtkImage = pImage;
}

void ImageWriter::SetFilename(const std::string& filename)
{
    mFilepath = filename;
}

void ImageWriter::Write()
{
    if(mFilepath == "")
    {
        EXCEPTION("Output file not specified for image writer.");
    }

    if(!mpVtkImage)
    {
        EXCEPTION("Output image not set for image writer.");
    }

    if(PetscTools::AmMaster())
    {
        vtkSmartPointer<vtkXMLImageDataWriter> p_writer1 = vtkSmartPointer<vtkXMLImageDataWriter>::New();
        p_writer1->SetFileName(mFilepath.c_str());
        #if VTK_MAJOR_VERSION <= 5
            p_writer1->SetInput(mpVtkImage);
        #else
            p_writer1->SetInputData(mpVtkImage);
        #endif
        p_writer1->Update();
        p_writer1->Write();
    }
}
