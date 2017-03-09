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

#include <vtkFeatureEdges.h>
#include <vtkCleanPolyData.h>
#include <vtkTriangleFilter.h>
#include <vtkStripper.h>
#include <vtkSplineFilter.h>
#include <vtkConnectivityFilter.h>
#include <vtkGeometryFilter.h>
#include "Exception.hpp"
#include "BoundaryExtractor.hpp"

BoundaryExtractor::BoundaryExtractor()
    : mpInputSurface(),
      mpOutputSurface(),
      mSmoothingLength(1),
      mDoSmoothing(true)
{

}

boost::shared_ptr<BoundaryExtractor> BoundaryExtractor::Create()
{
    MAKE_PTR(BoundaryExtractor, pSelf);
    return pSelf;
}

BoundaryExtractor::~BoundaryExtractor()
{

}

vtkSmartPointer<vtkPolyData> BoundaryExtractor::GetOutput()
{
    if(mpOutputSurface)
    {
        return(mpOutputSurface);
    }
    else
    {
        EXCEPTION("No output set. Did you run 'Update()' ?");
    }
}

void BoundaryExtractor::SetInput(vtkSmartPointer<vtkPolyData> pInput)
{
    mpInputSurface = pInput;
}

void BoundaryExtractor::SetSmoothingLength(double value)
{
    mSmoothingLength = value;
}

void BoundaryExtractor::SetDoSmoothing(bool doSmoothing)
{
    mDoSmoothing = doSmoothing;
}

void BoundaryExtractor::Update()
{
    if(!mpInputSurface)
    {
        EXCEPTION("No input set.");
    }

    vtkSmartPointer<vtkFeatureEdges> p_features = vtkSmartPointer<vtkFeatureEdges>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_features->SetInput(mpInputSurface);
    #else
        p_features->SetInputData(mpInputSurface);
    #endif

    vtkSmartPointer<vtkCleanPolyData> p_clean = vtkSmartPointer<vtkCleanPolyData>::New();
    p_clean->SetInputConnection(p_features->GetOutputPort());

    vtkSmartPointer<vtkTriangleFilter> p_triangle = vtkSmartPointer<vtkTriangleFilter>::New();
    p_triangle->SetInputConnection(p_clean->GetOutputPort());
    p_triangle->Update();

    if(mDoSmoothing)
    {
        vtkSmartPointer<vtkStripper> p_stripper = vtkSmartPointer<vtkStripper>::New();
        p_stripper->SetInputConnection(p_triangle->GetOutputPort());

        vtkSmartPointer<vtkSplineFilter> p_spline = vtkSmartPointer<vtkSplineFilter>::New();
        p_spline->SetInputConnection(p_stripper->GetOutputPort());
        p_spline->SetSubdivideToLength();
        p_spline->SetLength(mSmoothingLength);
        p_spline->Update();

        vtkSmartPointer<vtkGeometryFilter> p_geom = vtkSmartPointer<vtkGeometryFilter>::New();
        p_geom->SetInputConnection(p_spline->GetOutputPort());
        p_geom->Update();

        vtkSmartPointer<vtkTriangleFilter> p_triangle2 = vtkSmartPointer<vtkTriangleFilter>::New();
        p_triangle2->SetInputConnection(p_geom->GetOutputPort());
        p_triangle2->Update();

        vtkSmartPointer<vtkCleanPolyData> p_clean2 = vtkSmartPointer<vtkCleanPolyData>::New();
        p_clean2->SetInputConnection(p_triangle2->GetOutputPort());
        p_clean2->Update();

        mpOutputSurface = p_clean2->GetOutput();
    }
    else
    {
        mpOutputSurface = p_triangle->GetOutput();
    }
}
