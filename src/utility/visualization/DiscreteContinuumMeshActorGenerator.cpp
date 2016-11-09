/*

Copyright (c) 2005-2016, University of Oxford.
 All rights reserved.

 University of Oxford means the Chancellor, Masters and Scholars of the
 University of Oxford, having an administrative office at Wellington
 Square, Oxford OX1 2JD, UK.

 This file is DiscreteContinuumMesh of Chaste.

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
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A DiscreteContinuumMeshICULAR PURPOSE
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
#include "VesselNetworkWriter.hpp"

#include "DiscreteContinuumMeshActorGenerator.hpp"


template<unsigned DIM>
DiscreteContinuumMeshActorGenerator<DIM>::DiscreteContinuumMeshActorGenerator()
    : AbstractActorGenerator<DIM>(),
      mpDiscreteContinuumMesh()
{

}

template<unsigned DIM>
DiscreteContinuumMeshActorGenerator<DIM>::~DiscreteContinuumMeshActorGenerator()
{

}

template<unsigned DIM>
void DiscreteContinuumMeshActorGenerator<DIM>::AddActor(vtkSmartPointer<vtkRenderer> pRenderer)
{
    if(mpDiscreteContinuumMesh)
    {
        vtkSmartPointer<vtkUnstructuredGrid> p_grid = mpDiscreteContinuumMesh->GetAsVtkUnstructuredGrid();

        vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_geom_filter->SetInput(p_grid);
        #else
            p_geom_filter->SetInputData(p_grid);
        #endif

        vtkSmartPointer<vtkPolyDataMapper> p_mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_mapper->SetInput(p_geom_filter->GetOutput());
        #else
            p_mapper->SetInputData(p_geom_filter->GetOutput());
        #endif

        vtkSmartPointer<vtkActor> p_actor = vtkSmartPointer<vtkActor>::New();
        p_actor->SetMapper(p_mapper);
        p_actor->GetProperty()->SetColor(0,0,1);
        pRenderer->AddActor(p_actor);
    }
}

template<unsigned DIM>
void DiscreteContinuumMeshActorGenerator<DIM>::SetDiscreteContinuumMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > pDiscreteContinuumMesh)
{
    mpDiscreteContinuumMesh = pDiscreteContinuumMesh;
}

template class DiscreteContinuumMeshActorGenerator<2>;
template class DiscreteContinuumMeshActorGenerator<3>;