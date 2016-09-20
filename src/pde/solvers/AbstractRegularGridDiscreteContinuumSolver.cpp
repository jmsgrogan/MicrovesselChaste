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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkProbeFilter.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include "RegularGridWriter.hpp"

#include "AbstractRegularGridDiscreteContinuumSolver.hpp"

template<unsigned DIM>
AbstractRegularGridDiscreteContinuumSolver<DIM>::AbstractRegularGridDiscreteContinuumSolver()
    :   AbstractDiscreteContinuumSolver<DIM>(),
        mpVtkSolution(),
        mpRegularGrid()
{
    this->mHasRegularGrid = true;
}

template<unsigned DIM>
AbstractRegularGridDiscreteContinuumSolver<DIM>::~AbstractRegularGridDiscreteContinuumSolver()
{

}

template<unsigned DIM>
boost::shared_ptr<RegularGrid<DIM> > AbstractRegularGridDiscreteContinuumSolver<DIM>::GetGrid()
{
    if(!this->mpRegularGrid)
    {
        EXCEPTION("A regular grid has not been set.");
    }
    return this->mpRegularGrid;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractRegularGridDiscreteContinuumSolver<DIM>::GetConcentrations(const std::vector<DimensionalChastePoint<DIM> >& samplePoints)
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    std::vector<units::quantity<unit::concentration> > sampled_solution(samplePoints.size(), 0.0*unit::mole_per_metre_cubed);

    // Sample the field at these locations
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    p_points->SetNumberOfPoints(samplePoints.size());
    for(unsigned idx=0; idx< samplePoints.size(); idx++)
    {
        if(DIM==3)
        {
            p_points->SetPoint(idx, samplePoints[idx][0], samplePoints[idx][1], samplePoints[idx][2]);
        }
        else
        {
            p_points->SetPoint(idx, samplePoints[idx][0], samplePoints[idx][1], 0.0);
        }
    }
    p_polydata->SetPoints(p_points);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    p_probe_filter->SetInputData(p_polydata);
    p_probe_filter->SetSourceData(this->mpVtkSolution);
    p_probe_filter->Update();
    vtkSmartPointer<vtkPointData> p_point_data = p_probe_filter->GetOutput()->GetPointData();

    unsigned num_points = p_point_data->GetArray(this->mLabel.c_str())->GetNumberOfTuples();
    for(unsigned idx=0; idx<num_points; idx++)
    {
        sampled_solution[idx] = p_point_data->GetArray(this->mLabel.c_str())->GetTuple1(idx)*unit::mole_per_metre_cubed;
    }
    return sampled_solution;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractRegularGridDiscreteContinuumSolver<DIM>::GetConcentrations(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    if(this->mpRegularGrid == pGrid)
    {
        return this->GetConcentrations();
    }
    else
    {
        return this->GetConcentrations(pGrid->GetLocations());
    }
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractRegularGridDiscreteContinuumSolver<DIM>::GetConcentrations(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh)
{
    return this->GetConcentrations(pMesh->GetNodeLocationsAsPoints());
}

template<unsigned DIM>
std::vector<double> AbstractRegularGridDiscreteContinuumSolver<DIM>::GetSolution(const std::vector<DimensionalChastePoint<DIM> >& rSamplePoints)
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    std::vector<double> sampled_solution(rSamplePoints.size(), 0.0);

    // Sample the field at these locations
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    p_points->SetNumberOfPoints(rSamplePoints.size());
    for(unsigned idx=0; idx< rSamplePoints.size(); idx++)
    {
        if(DIM==3)
        {
            p_points->SetPoint(idx, rSamplePoints[idx][0], rSamplePoints[idx][1], rSamplePoints[idx][2]);
        }
        else
        {
            p_points->SetPoint(idx, rSamplePoints[idx][0], rSamplePoints[idx][1], 0.0);
        }
    }
    p_polydata->SetPoints(p_points);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    p_probe_filter->SetInputData(p_polydata);
    p_probe_filter->SetSourceData(this->mpVtkSolution);
    p_probe_filter->Update();
    vtkSmartPointer<vtkPointData> p_point_data = p_probe_filter->GetOutput()->GetPointData();

    unsigned num_points = p_point_data->GetArray(this->mLabel.c_str())->GetNumberOfTuples();
    for(unsigned idx=0; idx<num_points; idx++)
    {
        sampled_solution[idx] = p_point_data->GetArray(this->mLabel.c_str())->GetTuple1(idx);
    }
    return sampled_solution;
}

template<unsigned DIM>
std::vector<double> AbstractRegularGridDiscreteContinuumSolver<DIM>::GetSolution(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    if(this->mpRegularGrid == pGrid)
    {
        return this->GetSolution();
    }
    else
    {
        return this->GetSolution(pGrid->GetLocations());
    }
}

template<unsigned DIM>
std::vector<double> AbstractRegularGridDiscreteContinuumSolver<DIM>::GetSolution(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh)
{
    return this->GetSolution(pMesh->GetNodeLocationsAsPoints());
}

template<unsigned DIM>
vtkSmartPointer<vtkImageData> AbstractRegularGridDiscreteContinuumSolver<DIM>::GetVtkSolution()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }
    return this->mpVtkSolution;
}

template<unsigned DIM>
void AbstractRegularGridDiscreteContinuumSolver<DIM>::SetGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    this->mpRegularGrid = pGrid;
}

template<unsigned DIM>
void AbstractRegularGridDiscreteContinuumSolver<DIM>::Setup()
{
    if(!this->mpRegularGrid)
    {
        EXCEPTION("Regular grid DiscreteContinuum solvers need a grid before Setup can be called.");
    }

    // Set up the VTK solution
    this->mpVtkSolution = vtkSmartPointer<vtkImageData>::New();

    if(DIM==3)
    {
        this->mpVtkSolution->SetDimensions(this->mpRegularGrid->GetExtents()[0], this->mpRegularGrid->GetExtents()[1], this->mpRegularGrid->GetExtents()[2]);
    }
    else
    {
        this->mpVtkSolution->SetDimensions(this->mpRegularGrid->GetExtents()[0], this->mpRegularGrid->GetExtents()[1], 1);
    }

    double spacing = this->mpRegularGrid->GetSpacing()/this->mpRegularGrid->GetReferenceLengthScale();
    this->mpVtkSolution->SetSpacing(spacing, spacing, spacing);

    if(DIM==3)
    {
        this->mpVtkSolution->SetOrigin(this->mpRegularGrid->GetOrigin()[0], this->mpRegularGrid->GetOrigin()[1], this->mpRegularGrid->GetOrigin()[2]);
    }
    else
    {
        this->mpVtkSolution->SetOrigin(this->mpRegularGrid->GetOrigin()[0], this->mpRegularGrid->GetOrigin()[1], 0.0);
    }

    this->mSolution = std::vector<double>(0.0, this->mpRegularGrid->GetNumberOfPoints());
}

template<unsigned DIM>
void AbstractRegularGridDiscreteContinuumSolver<DIM>::UpdateSolution(std::vector<double>& data)
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    vtkSmartPointer<vtkDoubleArray> pPointData = vtkSmartPointer<vtkDoubleArray>::New();
    pPointData->SetNumberOfComponents(1);
    pPointData->SetNumberOfTuples(data.size());
    pPointData->SetName(this->GetLabel().c_str());
    for (unsigned i = 0; i < data.size(); i++)
    {
        pPointData->SetValue(i, data[i]);
    }
    this->mpVtkSolution->GetPointData()->AddArray(pPointData);

    // Note, if the data vector being passed in is mPointSolution, then it will be overwritten with zeros.
    this->mSolution = std::vector<double>(data.size(), 0.0);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mSolution[i] = data[i];
    }
}

template<unsigned DIM>
void AbstractRegularGridDiscreteContinuumSolver<DIM>::UpdateSolution(std::vector<units::quantity<unit::concentration> >& data)
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    vtkSmartPointer<vtkDoubleArray> pPointData = vtkSmartPointer<vtkDoubleArray>::New();
    pPointData->SetNumberOfComponents(1);
    pPointData->SetNumberOfTuples(data.size());
    pPointData->SetName(this->GetLabel().c_str());
    for (unsigned i = 0; i < data.size(); i++)
    {
        pPointData->SetValue(i, data[i]/unit::mole_per_metre_cubed);
    }
    this->mpVtkSolution->GetPointData()->AddArray(pPointData);

    // Note, if the data vector being passed in is mPointSolution, then it will be overwritten with zeros.
    this->mConcentrations = std::vector<units::quantity<unit::concentration> >(data.size(), 0.0*unit::mole_per_metre_cubed);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mConcentrations[i] = data[i];
    }
}

template<unsigned DIM>
void AbstractRegularGridDiscreteContinuumSolver<DIM>::UpdateCellData()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    if(!this->CellPopulationIsSet())
    {
        EXCEPTION("The DiscreteContinuum solver needs a cell population for this operation.");
    }

    this->mpRegularGrid->SetCellPopulation(*(this->mpCellPopulation));
    std::vector<std::vector<CellPtr> > point_cell_map = this->mpRegularGrid->GetPointCellMap();
    for(unsigned idx=0; idx<point_cell_map.size(); idx++)
    {
        for(unsigned jdx=0; jdx<point_cell_map[idx].size(); jdx++)
        {
            point_cell_map[idx][jdx]->GetCellData()->SetItem(this->mLabel, this->mSolution[idx]);
        }
    }
}

template<unsigned DIM>
void AbstractRegularGridDiscreteContinuumSolver<DIM>::Update()
{

}

template<unsigned DIM>
void AbstractRegularGridDiscreteContinuumSolver<DIM>::Write()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    if(!this->mpOutputFileHandler)
    {
        EXCEPTION("An output file handler has not been set for the DiscreteContinuum solver.");
    }

    RegularGridWriter writer;
    if(!this->mFilename.empty())
    {
        writer.SetFilename((this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "/" + this->mFilename));
    }
    else
    {
        writer.SetFilename((this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "/solution.vti"));
    }
    writer.SetImage(this->mpVtkSolution);
    writer.Write();
}

// Explicit instantiation
template class AbstractRegularGridDiscreteContinuumSolver<2> ;
template class AbstractRegularGridDiscreteContinuumSolver<3> ;
