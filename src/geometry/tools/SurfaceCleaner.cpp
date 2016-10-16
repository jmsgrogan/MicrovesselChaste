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

#include <vtkDecimatePro.h>
#include <vtkLinearSubdivisionFilter.h>
#include "Exception.hpp"
#include "SurfaceCleaner.hpp"

SurfaceCleaner::SurfaceCleaner()
    : mpInputSurface(),
      mpOutputSurface(),
      mDecimateTargetReduction(0.9994),
      mDecimateFeatureAngle(15.0),
      mLinearSubdivisionNumber(3)
{

}

boost::shared_ptr<SurfaceCleaner> SurfaceCleaner::Create()
{
    MAKE_PTR(SurfaceCleaner, pSelf);
    return pSelf;
}

SurfaceCleaner::~SurfaceCleaner()
{

}

vtkSmartPointer<vtkPolyData> SurfaceCleaner::GetOutput()
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

void SurfaceCleaner::SetInput(vtkSmartPointer<vtkPolyData> pInput)
{
    mpInputSurface = pInput;
}

void SurfaceCleaner::SetDecimateTargetReduction(double value)
{
    mDecimateTargetReduction = value;
}

void SurfaceCleaner::SetDecimateFeatureAngle(double value)
{
    mDecimateFeatureAngle = value;
}

void SurfaceCleaner::SetLinearSubdivisionNumber(double value)
{
    mLinearSubdivisionNumber = value;
}

void SurfaceCleaner::Update()
{
    if(!mpInputSurface)
    {
        EXCEPTION("No input set.");
    }

    vtkSmartPointer<vtkDecimatePro> p_decimate = vtkSmartPointer<vtkDecimatePro>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_decimate->SetInput(mpInputSurface);
    #else
        p_decimate->SetInputData(mpInputSurface);
    #endif
    p_decimate->SetTargetReduction(mDecimateTargetReduction);
    p_decimate->SetFeatureAngle(mDecimateFeatureAngle);
    p_decimate->Update();

    vtkSmartPointer<vtkLinearSubdivisionFilter> p_divide = vtkSmartPointer<vtkLinearSubdivisionFilter>::New();
    p_divide->SetInputConnection(p_decimate->GetOutputPort());
    p_divide->SetNumberOfSubdivisions(mLinearSubdivisionNumber);
    p_divide->Update();

    mpOutputSurface = p_divide->GetOutput();
}
