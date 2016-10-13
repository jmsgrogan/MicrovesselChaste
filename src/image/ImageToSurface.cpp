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
#include "ImageToSurface.hpp"
#include <vtkThreshold.h>
#include <vtkGeometryFilter.h>
#include <vtkMarchingCubes.h>
#include <vtkTriangleFilter.h>
#include <Debug.hpp>

ImageToSurface::ImageToSurface()
    : mpImage(vtkSmartPointer<vtkImageData>::New()),
      mSegmentAboveThreshold(true),
      mThreshold(1.0),
      mpSurface(),
      mUseMarchingCubes(false)
{

}

boost::shared_ptr<ImageToSurface> ImageToSurface::Create()
{
    MAKE_PTR(ImageToSurface, pSelf);
    return pSelf;
}

ImageToSurface::~ImageToSurface()
{

}

vtkSmartPointer<vtkPolyData> ImageToSurface::GetOutput()
{
    if(mpSurface)
    {
        return(mpSurface);
    }
    else
    {
        EXCEPTION("No output set. Did you run 'Update()' ?");
    }
}

void ImageToSurface::SetInput(vtkSmartPointer<vtkImageData> pImage)
{
    mpImage = pImage;
}

void ImageToSurface::SetInputRaw(vtkImageData* pImage)
{
    mpImage.TakeReference(pImage);
//    pImage->Delete();
}

void ImageToSurface::SetThreshold(double threshold, bool segmentAboveThreshold)
{
    mThreshold = threshold;
    mSegmentAboveThreshold = segmentAboveThreshold;
}

void ImageToSurface::SetUseMarchingCubes(bool useMarchingCubes)
{
    mUseMarchingCubes = useMarchingCubes;
}

void ImageToSurface::Update()
{
    if(!mpImage)
    {
        EXCEPTION("No input set.");
    }

    if(mUseMarchingCubes)
    {
        vtkSmartPointer<vtkMarchingCubes> p_marching_cubes = vtkSmartPointer<vtkMarchingCubes>::New();
        p_marching_cubes->SetInputData(mpImage);
        p_marching_cubes->SetValue(0, mThreshold);
        p_marching_cubes->Update();

        mpSurface = p_marching_cubes->GetOutput();
    }
    else
    {
        vtkSmartPointer<vtkThreshold> p_threshold = vtkSmartPointer<vtkThreshold>::New();
        p_threshold->SetInputData(mpImage);

        if(mSegmentAboveThreshold)
        {
            p_threshold->ThresholdByUpper(mThreshold);
        }
        else
        {
            p_threshold->ThresholdByLower(mThreshold);
        }

        vtkSmartPointer<vtkGeometryFilter> p_geometry = vtkSmartPointer<vtkGeometryFilter>::New();
        p_geometry->SetInputConnection(p_threshold->GetOutputPort());
        p_geometry->Update();

        vtkSmartPointer<vtkTriangleFilter> p_triangle = vtkSmartPointer<vtkTriangleFilter>::New();
        p_triangle->SetInputConnection(p_geometry->GetOutputPort());
        p_triangle->Update();

        mpSurface = p_triangle->GetOutput();
    }
}
