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

#include "Exception.hpp"
#include "ImageToMesh.hpp"
#include <vtkThreshold.h>
#include <vtkGeometryFilter.h>
#include <vtkMarchingCubes.h>
#include <vtkTriangleFilter.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkProbeFilter.h>
#include "ImageToSurface.hpp"
#include "SurfaceCleaner.hpp"
#include "BoundaryExtractor.hpp"
#include "UblasIncludes.hpp"
#include "GeometryWriter.hpp"
#include "DiscreteContinuumMeshGenerator.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
ImageToMesh<DIM>::ImageToMesh()
    : mpImage(vtkSmartPointer<vtkImageData>::New()),
      mElementSize(0.0 * unit::metres * unit::metres* unit::metres),
      mMesh(),
      mpDomain()
{

}

template<unsigned DIM>
boost::shared_ptr<ImageToMesh<DIM> > ImageToMesh<DIM>::Create()
{
    MAKE_PTR(ImageToMesh, pSelf);
    return pSelf;
}

template<unsigned DIM>
ImageToMesh<DIM>::~ImageToMesh()
{

}

template<unsigned DIM>
boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > ImageToMesh<DIM>::GetMesh()
{
    if(mMesh)
    {
        return(mMesh);
    }
    else
    {
        EXCEPTION("No mesh set. Did you run 'Update()' ?");
    }
}

template<unsigned DIM>
void ImageToMesh<DIM>::SetTissueDomain(boost::shared_ptr<Part<DIM> > pTissueDomain)
{
    mpDomain = pTissueDomain;
}

template<unsigned DIM>
void ImageToMesh<DIM>::SetElementSize(units::quantity<unit::volume> elementSize)
{
    mElementSize = elementSize;
}

template<unsigned DIM>
void ImageToMesh<DIM>::SetInput(vtkSmartPointer<vtkImageData> pImage)
{
    mpImage = pImage;
}

template<unsigned DIM>
void ImageToMesh<DIM>::SetInputRaw(vtkImageData* pImage)
{
    mpImage.TakeReference(pImage);
//    pImage->Delete();
}

template<unsigned DIM>
void ImageToMesh<DIM>::Update()
{
    if(!mpImage)
    {
        EXCEPTION("No input set.");
    }

    ImageToSurface image_to_surface;
    image_to_surface.SetInput(mpImage);
    image_to_surface.SetThreshold(1.0, false);
    image_to_surface.Update();

    BoundaryExtractor extractor;
    extractor.SetInput(image_to_surface.GetOutput());
    extractor.SetDoSmoothing(true);
    extractor.SetSmoothingLength(10.0);
    extractor.Update();

    if(DIM==2)
    {
        DiscreteContinuumMeshGenerator<DIM, DIM> temp_mesh_generator;
        temp_mesh_generator.SetDomain(extractor.GetOutput());
        if(mpDomain)
        {
            temp_mesh_generator.SetDomain(mpDomain);
        }
        temp_mesh_generator.Update();
        boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > p_temp_mesh = temp_mesh_generator.GetMesh();

        vtkSmartPointer<vtkPoints> mesh_points = p_temp_mesh->GetNodeLocations();
        std::vector<std::vector<unsigned> > mesh_connectivity = p_temp_mesh->GetConnectivity();

        units::quantity<unit::length> reference_length = p_temp_mesh->GetReferenceLengthScale();
        vtkSmartPointer<vtkPoints> p_vtk_points = vtkSmartPointer<vtkPoints>::New();
        for(unsigned idx = 0; idx<mesh_connectivity.size(); idx++)
        {
            double location0[3];
            double location1[3];
            double location2[3];
            mesh_points->GetPoint(mesh_connectivity[idx][0], location0);
            mesh_points->GetPoint(mesh_connectivity[idx][1], location1);
            mesh_points->GetPoint(mesh_connectivity[idx][2], location2);
            double centx = (location0[0] + location1[0] + location2[0])/3.0;
            double centy = (location0[1] + location1[1] + location2[1])/3.0;
            p_vtk_points->InsertNextPoint(centx, centy, 0.0);
        }

        // Get the value of the vessel from the image to determine if the point is inside or outside a vessel
        vtkSmartPointer<vtkPolyData> p_sampling_data = vtkSmartPointer<vtkPolyData>::New();
        p_sampling_data->SetPoints(p_vtk_points);

        vtkSmartPointer<vtkProbeFilter> p_image_probe = vtkSmartPointer<vtkProbeFilter>::New();
        #if VTK_MAJOR_VERSION <= 5
            p_image_probe->SetInput(p_sampling_data);
            p_image_probe->SetSource(mpImage);
        #else
            p_image_probe->SetInputData(p_sampling_data);
            p_image_probe->SetSourceData(mpImage);
        #endif
        p_image_probe->Update();

        std::vector<DimensionalChastePoint<DIM> > holes;
        std::vector<DimensionalChastePoint<DIM> > regions;
        for(unsigned idx=0; idx<p_image_probe->GetOutput()->GetPointData()->GetScalars()->GetNumberOfTuples(); idx++)
        {
            if(p_image_probe->GetOutput()->GetPointData()->GetScalars()->GetTuple1(idx)>5)
            {
                if(mpDomain)
                {
                    c_vector<double, DIM> loc;
                    loc[0] = p_vtk_points->GetPoint(idx)[0];
                    loc[1] = p_vtk_points->GetPoint(idx)[1];
                    if(mpDomain->GetPolygons()[0]->ContainsPoint(DimensionalChastePoint<DIM>(loc, reference_length)))
                    {
                        regions.push_back(DimensionalChastePoint<DIM>(loc, reference_length));
                    }
                    else
                    {
                        holes.push_back(DimensionalChastePoint<DIM>(loc, reference_length));
                    }
                }
                else
                {
                    c_vector<double, DIM> loc;
                    loc[0] = p_vtk_points->GetPoint(idx)[0];
                    loc[1] = p_vtk_points->GetPoint(idx)[1];
                    holes.push_back(DimensionalChastePoint<DIM>(loc, reference_length));
                }
            }
        }
        DiscreteContinuumMeshGenerator<DIM, DIM> fine_mesh_generator;
        fine_mesh_generator.SetDomain(extractor.GetOutput());
        fine_mesh_generator.SetMaxElementArea(mElementSize);
        fine_mesh_generator.SetHoles(holes);
        fine_mesh_generator.SetRegionMarkers(regions);
        if(mpDomain)
        {
            fine_mesh_generator.SetDomain(mpDomain);
        }
        fine_mesh_generator.Update();
        boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > p_fine_mesh = fine_mesh_generator.GetMesh();
        mMesh = p_fine_mesh;
    }
    else
    {
        EXCEPTION("3d image meshing not yet implemented");
    }
}

// Explicit instantiation
template class ImageToMesh<2> ;
template class ImageToMesh<3> ;
