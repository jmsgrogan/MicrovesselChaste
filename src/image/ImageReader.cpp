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
#include <vtkTIFFReader.h>
#include <vtkPNGReader.h>
#include <vtkJPEGReader.h>
#include <vtkBMPReader.h>
#include <vtkImageResize.h>
#include <vtkImageLuminance.h>
#include "ImageReader.hpp"

ImageReader::ImageReader()
    : mpVtkImage(),
      mFilepath(),
      mResizeX(1.0),
      mResizeY(1.0),
      mResizeZ(1.0)
{

}

boost::shared_ptr<ImageReader> ImageReader::Create()
{
    MAKE_PTR(ImageReader, pSelf);
    return pSelf;
}

ImageReader::~ImageReader()
{

}


vtkSmartPointer<vtkImageData> ImageReader::GetImage()
{
    if(mpVtkImage)
    {
        return mpVtkImage;
    }
    else
    {
        EXCEPTION("No image data has been set.");
    }
}

void ImageReader::SetFilename(const std::string& filename)
{
    mFilepath = filename;
}

void ImageReader::SetImageResizeFactors(double factorX, double factorY, double factorZ)
{
    mResizeX = factorX;
    mResizeY = factorY;
    mResizeZ = factorZ;
}

void ImageReader::Read()
{
    // Get the file extension
    if(mFilepath == "")
    {
        EXCEPTION("Input file not specified for image reader");
    }
    std::string file_extension  = boost::filesystem::extension(mFilepath);

    if(file_extension == ".tif" or file_extension == ".tiff" or file_extension == ".TIF" or file_extension == ".TIFF")
    {
        vtkSmartPointer<vtkTIFFReader> p_reader = vtkSmartPointer<vtkTIFFReader>::New();
        p_reader->SetFileName(mFilepath.c_str());
        if(mResizeX==1.0 and mResizeY==1.0 and mResizeZ==1.0)
        {
            p_reader->Update();
            mpVtkImage = p_reader->GetOutput();
        }
        else
        {
            vtkSmartPointer<vtkImageResize> p_resize = vtkSmartPointer<vtkImageResize>::New();
            p_resize->SetInputConnection(p_reader->GetOutputPort());
            p_resize->SetMagnificationFactors(mResizeX, mResizeY, mResizeZ);
            p_resize->SetResizeMethodToMagnificationFactors();
            p_resize->Update();
            mpVtkImage = p_resize->GetOutput();
        }
    }
    else if(file_extension == ".png" or file_extension == ".PNG")
    {
        vtkSmartPointer<vtkPNGReader> p_reader = vtkSmartPointer<vtkPNGReader>::New();
        p_reader->SetFileName(mFilepath.c_str());
        if(mResizeX==1.0 and mResizeY==1.0 and mResizeZ==1.0)
        {
            p_reader->Update();

            // RGB to scalar
            vtkSmartPointer<vtkImageLuminance> p_lum = vtkSmartPointer<vtkImageLuminance>::New();
            p_lum->SetInputConnection(p_reader->GetOutputPort());
            p_lum->Update();
            mpVtkImage = p_lum->GetOutput();
        }
        else
        {
            vtkSmartPointer<vtkImageResize> p_resize = vtkSmartPointer<vtkImageResize>::New();
            p_resize->SetInputConnection(p_reader->GetOutputPort());
            p_resize->SetMagnificationFactors(mResizeX, mResizeY, mResizeZ);
            p_resize->SetResizeMethodToMagnificationFactors();
            p_resize->Update();

            // RGB to scalar
            vtkSmartPointer<vtkImageLuminance> p_lum = vtkSmartPointer<vtkImageLuminance>::New();
            p_lum->SetInputConnection(p_resize->GetOutputPort());
            p_lum->Update();
            mpVtkImage = p_lum->GetOutput();
        }
    }
    else if(file_extension == ".jpg" or file_extension == ".JPG")
    {
        vtkSmartPointer<vtkJPEGReader> p_reader = vtkSmartPointer<vtkJPEGReader>::New();
        p_reader->SetFileName(mFilepath.c_str());
        if(mResizeX==1.0 and mResizeY==1.0 and mResizeZ==1.0)
        {
            p_reader->Update();
            mpVtkImage = p_reader->GetOutput();
        }
        else
        {
            vtkSmartPointer<vtkImageResize> p_resize = vtkSmartPointer<vtkImageResize>::New();
            p_resize->SetInputConnection(p_reader->GetOutputPort());
            p_resize->SetMagnificationFactors(mResizeX, mResizeY, mResizeZ);
            p_resize->SetResizeMethodToMagnificationFactors();
            p_resize->Update();
            mpVtkImage = p_resize->GetOutput();
        }
    }
    else if(file_extension == ".bmp" or file_extension == ".BMP")
    {
        vtkSmartPointer<vtkBMPReader> p_reader = vtkSmartPointer<vtkBMPReader>::New();
        p_reader->SetFileName(mFilepath.c_str());
        if(mResizeX==1.0 and mResizeY==1.0 and mResizeZ==1.0)
        {
            p_reader->Update();
            mpVtkImage = p_reader->GetOutput();
        }
        else
        {
            vtkSmartPointer<vtkImageResize> p_resize = vtkSmartPointer<vtkImageResize>::New();
            p_resize->SetInputConnection(p_reader->GetOutputPort());
            p_resize->SetMagnificationFactors(mResizeX, mResizeY, mResizeZ);
            p_resize->SetResizeMethodToMagnificationFactors();
            p_resize->Update();
            mpVtkImage = p_resize->GetOutput();
        }
    }
    else
    {
        EXCEPTION("Input file extension not recognized");
    }

    if(!mpVtkImage)
    {
        EXCEPTION("Image reading failed.");
    }
}
