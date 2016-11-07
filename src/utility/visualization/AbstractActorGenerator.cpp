/*

Copyright (c) 2005-2016, University of Oxford.
 All rights reserved.

 University of Oxford means the Chancellor, Masters and Scholars of the
 University of Oxford, having an administrative office at Wellington
 Square, Oxford OX1 2JD, UK.

 This file is Abstract of Chaste.

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
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A AbstractICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 */

#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>
#if VTK_MAJOR_VERSION > 5
    #include <vtkNamedColors.h>
#endif
#include <vtkSphereSource.h>
#include <vtkGlyph3D.h>
#include <vtkGlyph2D.h>
#include <vtkCubeAxesActor2D.h>
#include <vtkImageData.h>
#include <vtkGeometryFilter.h>
#include <vtkTubeFilter.h>
#include <vtkExtractEdges.h>
#include <vtkUnstructuredGrid.h>
#include <vtkCell.h>
#include <vtkPolygon.h>
#include <vtkIdList.h>
#include <vtkFeatureEdges.h>
#include "UblasIncludes.hpp"
#include "UblasVectorInclude.hpp"
#include "Exception.hpp"
#include "BaseUnits.hpp"

#include "AbstractActorGenerator.hpp"

template<unsigned DIM>
AbstractActorGenerator<DIM>::AbstractActorGenerator()
    : mpColorLookUpTable(vtkSmartPointer<vtkLookupTable>::New()),
      mLengthScale(BaseUnits::Instance()->GetReferenceLengthScale()),
      mShowEdges(true),
      mShowPoints(false),
      mShowVolume(true),
      mEdgeColor(zero_vector<double>(3)),
      mPointColor(unit_vector<double>(3, 2)),
      mVolumeColor(unit_vector<double>(3, 0)),
      mVolumeOpacity(0.6),
      mPointSize(1.0),
      mEdgeSize(0.5)
{
    mPointColor*=255.0;
    mVolumeColor*=255.0;
}

template<unsigned DIM>
AbstractActorGenerator<DIM>::~AbstractActorGenerator()
{

}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetShowEdges(bool show)
{
    mShowEdges = show;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetShowPoints(bool show)
{
    mShowPoints = show;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetPointSize(double size)
{
    mPointSize = size;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetEdgeSize(double size)
{
    mEdgeSize = size;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetShowVolume(bool show)
{
    mShowVolume = show;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetEdgeColor(const c_vector<double, 3>& rColor)
{
    mEdgeColor = rColor;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetPointColor(const c_vector<double, 3>& rColor)
{
    mPointColor = rColor;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetVolumeColor(const c_vector<double, 3>& rColor)
{
    mVolumeColor = rColor;
}

template<unsigned DIM>
void AbstractActorGenerator<DIM>::SetVolumeOpacity(double opacity)
{
    mVolumeOpacity = opacity;
}


template class AbstractActorGenerator<2>;
template class AbstractActorGenerator<3>;
