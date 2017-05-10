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

#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkDelaunay2D.h>
#include <vtkDelaunay3D.h>
#include <vtkSmartPointer.h>
#include <vtkGeometryFilter.h>
#include <vtkTriangleFilter.h>
#include <vtkMassProperties.h>
#include <vtkXMLPolyDataWriter.h>
#include "MeshBasedCellPopulation.hpp"
#include "VolumeOutputModifier.hpp"
#include "CancerCellMutationState.hpp"

template<unsigned DIM>
VolumeOutputModifier<DIM>::VolumeOutputModifier()
    : AbstractCellBasedSimulationModifier<DIM>(),
      mCellLengthScale(40.0e-6), // metre
      mTimeScale(1.0), //hours
      mOutputFileStream(),
      mOutputFrequency(1),
      mTumourVolumeOnly(false)
{
}

template<unsigned DIM>
VolumeOutputModifier<DIM>::~VolumeOutputModifier()
{
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetCellLengthScale(double cellLengthScale)
{
    mCellLengthScale = cellLengthScale;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetUseTumourVolumeOnly(bool tumourVolumeOnly)
{
    mTumourVolumeOnly = tumourVolumeOnly;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetOutputFrequency(unsigned outputFrequency)
{
    mOutputFrequency = outputFrequency;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetTimeScale(double timeScale)
{
    mTimeScale = timeScale;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetOutputFileStream(boost::shared_ptr<std::ofstream> ofstream)
{
    mOutputFileStream = ofstream;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    /*
     * We must update CellData in SetupSolve(), otherwise it will not have been
     * fully initialised by the time we enter the main time loop.
     */
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Make sure the cell population is updated
    rCellPopulation.Update();

    if(SimulationTime::Instance()->GetTimeStepsElapsed()%mOutputFrequency==0)
    {
        if(mTumourVolumeOnly)
        {
            vtkSmartPointer<vtkPoints> p_vtk_points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkPolyData> p_vtk_polydata = vtkSmartPointer<vtkPolyData>::New();
            vtkSmartPointer<vtkXMLPolyDataWriter> p_writer = vtkSmartPointer<vtkXMLPolyDataWriter>::New();
            // Loop through cells
            bool tumour_found = false;
            for (typename AbstractCellPopulation<DIM>::Iterator cell_iter = rCellPopulation.Begin();
                 cell_iter != rCellPopulation.End();
                 ++cell_iter)
            {
                if((*cell_iter)->template HasCellProperty<CancerCellMutationState>())
                {
                    c_vector<double, DIM> location = rCellPopulation.GetLocationOfCellCentre(*cell_iter);
                    if(DIM==2)
                    {
                        p_vtk_points->InsertNextPoint(location[0], location[1], 0.0);
                    }
                    else
                    {
                        p_vtk_points->InsertNextPoint(location[0], location[1], location[2]);
                    }
                    tumour_found = true;
                }
            }
            double total_volume = 0.0;
            if(tumour_found)
            {
                p_vtk_polydata->SetPoints(p_vtk_points);

                vtkSmartPointer<vtkGeometryFilter> p_geom_filter = vtkSmartPointer<vtkGeometryFilter>::New();
                vtkSmartPointer<vtkTriangleFilter> p_tri_filter = vtkSmartPointer<vtkTriangleFilter>::New();
                vtkSmartPointer<vtkMassProperties> p_mass_prop = vtkSmartPointer<vtkMassProperties>::New();

                if(DIM==2)
                {
                    vtkSmartPointer<vtkDelaunay2D> p_vtk_delaunay= vtkSmartPointer<vtkDelaunay2D>::New();
                    #if VTK_MAJOR_VERSION <= 5
                    p_vtk_delaunay->SetInput(p_vtk_polydata);
                    #else
                    p_vtk_delaunay->SetInputData(p_vtk_polydata);
                    #endif
                    p_geom_filter->SetInputConnection(p_vtk_delaunay->GetOutputPort());
                }
                else
                {
                    vtkSmartPointer<vtkDelaunay3D> p_vtk_delaunay= vtkSmartPointer<vtkDelaunay3D>::New();
                    #if VTK_MAJOR_VERSION <= 5
                    p_vtk_delaunay->SetInput(p_vtk_polydata);
                    #else
                    p_vtk_delaunay->SetInputData(p_vtk_polydata);
                    #endif
                    p_geom_filter->SetInputConnection(p_vtk_delaunay->GetOutputPort());
                }
                p_tri_filter->SetInputConnection(p_geom_filter->GetOutputPort());
                p_mass_prop->SetInputConnection(p_tri_filter->GetOutputPort());
                p_mass_prop->Update();
                if(DIM==2)
                {
                    total_volume = p_mass_prop->GetSurfaceArea(); // in metres
                }
                else
                {
                    total_volume = p_mass_prop->GetVolume(); // in metres
                }
            }
            if(mOutputFileStream->is_open())
            {
                double current_time = SimulationTime::Instance()->GetTime() * mTimeScale;
                double tumour_area = total_volume*(mCellLengthScale*mCellLengthScale);
                (*mOutputFileStream) << current_time << "," << tumour_area << std::endl;
            }
        }
        else
        {
            double total_volume = rCellPopulation.GetTetrahedralMeshForPdeModifier()->GetVolume(); // in metres
            if(mOutputFileStream->is_open())
            {
                double current_time = SimulationTime::Instance()->GetTime() * mTimeScale;
                double tumour_area = total_volume*(mCellLengthScale*mCellLengthScale);
                (*mOutputFileStream) << current_time << "," << tumour_area <<  std::endl;
            }
        }
    }
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class VolumeOutputModifier<1>;
template class VolumeOutputModifier<2>;
template class VolumeOutputModifier<3>;

