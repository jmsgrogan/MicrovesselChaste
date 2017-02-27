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
#include "AbstractUnstructuredGridDiscreteContinuumSolver.hpp"

template<unsigned DIM>
AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::AbstractUnstructuredGridDiscreteContinuumSolver()
    :   AbstractDiscreteContinuumSolver<DIM>(),
        mpVtkSolution(),
        mpMesh()
{
    this->mHasUnstructuredGrid = true;
}

template<unsigned DIM>
AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::~AbstractUnstructuredGridDiscreteContinuumSolver()
{

}

template<unsigned DIM>
boost::shared_ptr<DiscreteContinuumMesh<DIM> > AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetMesh()
{
    if(!this->mpMesh)
    {
        EXCEPTION("A mesh has not been set.");
    }
    return this->mpMesh;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetConcentrationsAtCentroids()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    std::vector<c_vector<double, DIM> > centroids = mpMesh->GetElementCentroids();
    std::vector<units::quantity<unit::concentration> > sampled_solution(centroids.size(), 0.0*this->mReferenceConcentration);

    // Sample the field at these locations
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();
    p_points->SetNumberOfPoints(centroids.size());
    for(unsigned idx=0; idx< centroids.size(); idx++)
    {
        if(DIM==3)
        {
            p_points->SetPoint(idx, centroids[idx][0], centroids[idx][1], centroids[idx][2]);
        }
        else
        {
            p_points->SetPoint(idx, centroids[idx][0], centroids[idx][1], 0.0);
        }
    }
    p_polydata->SetPoints(p_points);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_probe_filter->SetInput(p_polydata);
        p_probe_filter->SetSource(this->mpVtkSolution);
    #else
        p_probe_filter->SetInputData(p_polydata);
        p_probe_filter->SetSourceData(this->mpVtkSolution);
    #endif
    p_probe_filter->Update();
    vtkSmartPointer<vtkPointData> p_point_data = p_probe_filter->GetOutput()->GetPointData();

    unsigned num_points = p_point_data->GetArray(this->mLabel.c_str())->GetNumberOfTuples();
    for(unsigned idx=0; idx<num_points; idx++)
    {
        sampled_solution[idx] = p_point_data->GetArray(this->mLabel.c_str())->GetTuple1(idx)*this->mReferenceConcentration;
    }
    return sampled_solution;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetConcentrations(const std::vector<DimensionalChastePoint<DIM> >& samplePoints)
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    std::vector<units::quantity<unit::concentration> > sampled_solution(samplePoints.size(), 0.0*this->mReferenceConcentration);

    // Sample the field at these locations
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> p_points = vtkSmartPointer<vtkPoints>::New();

    p_points->SetNumberOfPoints(samplePoints.size());
    for(unsigned idx=0; idx< samplePoints.size(); idx++)
    {
        c_vector<double, DIM> location = samplePoints[idx].GetLocation(this->mpMesh->GetReferenceLengthScale());
        if(DIM==3)
        {
            p_points->SetPoint(idx, location[0], location[1], location[2]);
        }
        else
        {
            p_points->SetPoint(idx, location[0], location[1], 0.0);
        }
    }
    p_polydata->SetPoints(p_points);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_probe_filter->SetInput(p_polydata);
        p_probe_filter->SetSource(this->mpVtkSolution);
    #else
        p_probe_filter->SetInputData(p_polydata);
        p_probe_filter->SetSourceData(this->mpVtkSolution);
    #endif
    p_probe_filter->Update();
    vtkSmartPointer<vtkPointData> p_point_data = p_probe_filter->GetOutput()->GetPointData();

    unsigned num_points = p_point_data->GetArray(this->mLabel.c_str())->GetNumberOfTuples();
    for(unsigned idx=0; idx<num_points; idx++)
    {
        sampled_solution[idx] = p_point_data->GetArray(this->mLabel.c_str())->GetTuple1(idx)*this->mReferenceConcentration;
    }
    return sampled_solution;
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetConcentrations(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    return this->GetConcentrations(pGrid->GetGlobalLocations());
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetConcentrations(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh)
{
    if(this->mpMesh == pMesh)
    {
        return this->GetConcentrations();
    }
    else
    {
        return this->GetConcentrations(pMesh->GetNodeLocationsAsPoints());
    }
}

template<unsigned DIM>
std::vector<double> AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetSolution(const std::vector<DimensionalChastePoint<DIM> >& rSamplePoints)
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
        c_vector<double, DIM> location = rSamplePoints[idx].GetLocation(this->mpMesh->GetReferenceLengthScale());
        if(DIM==3)
        {
            p_points->SetPoint(idx, location[0], location[1], location[2]);
        }
        else
        {
            p_points->SetPoint(idx, location[0], location[1], 0.0);
        }
    }
    p_polydata->SetPoints(p_points);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_probe_filter->SetInput(p_polydata);
        p_probe_filter->SetSource(this->mpVtkSolution);
    #else
        p_probe_filter->SetInputData(p_polydata);
        p_probe_filter->SetSourceData(this->mpVtkSolution);
    #endif
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
std::vector<double> AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetSolution(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    return this->GetSolution(pGrid->GetGlobalLocations());
}

template<unsigned DIM>
std::vector<double> AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetSolution(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh)
{
    if(this->mpMesh == pMesh)
    {
        return this->GetSolution();
    }
    else
    {
        return this->GetSolution(pMesh->GetNodeLocationsAsPoints());
    }
}

template<unsigned DIM>
vtkSmartPointer<vtkUnstructuredGrid> AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::GetVtkSolution()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }
    return this->mpVtkSolution;
}

template<unsigned DIM>
void AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > pMesh)
{
    this->mpMesh = pMesh;
}

template<unsigned DIM>
void AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::Setup()
{
    if(!this->mpMesh)
    {
        EXCEPTION("Mesh needed before Setup can be called.");
    }

    // Set up the VTK solution
    this->mpVtkSolution = mpMesh->GetAsVtkUnstructuredGrid();

    unsigned num_nodes = this->mpMesh->GetNodeLocationsAsPoints().size();
    this->mSolution = std::vector<double>(0.0, num_nodes);

    if(this->mpNetwork)
    {
        this->mpMesh->SetVesselNetwork(this->mpNetwork)  ;
    }
}

template<unsigned DIM>
void AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::UpdateSolution(const std::vector<double>& data)
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
    this->mpMesh->SetNodalData(data);

    // Note, if the data vector being passed in is mPointSolution, then it will be overwritten with zeros.
    this->mSolution = std::vector<double>(data.size(), 0.0);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mSolution[i] = data[i];
    }
}

template<unsigned DIM>
void AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::UpdateElementSolution(const std::vector<double>& data)
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
    this->mpVtkSolution->GetCellData()->AddArray(pPointData);
}

template<unsigned DIM>
void AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::UpdateSolution(const std::vector<units::quantity<unit::concentration> >& data)
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
        pPointData->SetValue(i, data[i]/this->mReferenceConcentration);
    }
    this->mpVtkSolution->GetPointData()->AddArray(pPointData);

    // Note, if the data vector being passed in is mPointSolution, then it will be overwritten with zeros.
    this->mConcentrations = std::vector<units::quantity<unit::concentration> >(data.size(), 0.0*this->mReferenceConcentration);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mConcentrations[i] = data[i];
    }
}

template<unsigned DIM>
void AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::UpdateCellData()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    if(!this->CellPopulationIsSet())
    {
        EXCEPTION("The DiscreteContinuum solver needs a cell population for this operation.");
    }

    // Update for FEM
//    this->mpRegularGrid->SetCellPopulation(*(this->mpCellPopulation));
//    std::vector<std::vector<CellPtr> > point_cell_map = this->mpRegularGrid->GetPointCellMap();
//    for(unsigned idx=0; idx<point_cell_map.size(); idx++)
//    {
//        for(unsigned jdx=0; jdx<point_cell_map[idx].size(); jdx++)
//        {
//            point_cell_map[idx][jdx]->GetCellData()->SetItem(this->mLabel, this->mSolution[idx]);
//        }
//    }
}

template<unsigned DIM>
void AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::Update()
{

}

template<unsigned DIM>
void AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::Write()
{
    if(!this->mpVtkSolution)
    {
        this->Setup();
    }

    if(!this->mpOutputFileHandler)
    {
        EXCEPTION("An output file handler has not been set for the DiscreteContinuum solver.");
    }

    if(PetscTools::AmMaster())
    {
        vtkSmartPointer<vtkXMLUnstructuredGridWriter> p_writer1 = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
        if(!this->mFilename.empty())
        {
            p_writer1->SetFileName((this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "/" + this->mFilename+".vtu").c_str());
        }
        else
        {
            p_writer1->SetFileName((this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "/solution.vtu").c_str());
        }
        #if VTK_MAJOR_VERSION <= 5
            p_writer1->SetInput(this->mpVtkSolution);
        #else
            p_writer1->SetInputData(this->mpVtkSolution);
        #endif
        p_writer1->Update();
        p_writer1->Write();
    }
}

// Explicit instantiation
template class AbstractUnstructuredGridDiscreteContinuumSolver<2> ;
template class AbstractUnstructuredGridDiscreteContinuumSolver<3> ;
