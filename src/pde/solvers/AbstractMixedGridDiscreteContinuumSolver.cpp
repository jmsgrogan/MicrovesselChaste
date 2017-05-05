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

#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkProbeFilter.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include "AbstractDiscreteContinuumPde.hpp"
#include "AbstractMixedGridDiscreteContinuumSolver.hpp"
#include "RegularGridWriter.hpp"

template<unsigned DIM>
AbstractMixedGridDiscreteContinuumSolver<DIM>::AbstractMixedGridDiscreteContinuumSolver()
    :   AbstractDiscreteContinuumSolver<DIM>(),
        mpMesh(),
        mpRegularGrid()
{

}

template<unsigned DIM>
AbstractMixedGridDiscreteContinuumSolver<DIM>::~AbstractMixedGridDiscreteContinuumSolver()
{

}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractMixedGridDiscreteContinuumSolver<DIM>::GetConcentrationsAtCentroids()
{
    return this->GetConcentrations(this->mpDensityMap->GetGridCalculator()->GetGrid()->GetCellLocations());
}

template<unsigned DIM>
void AbstractMixedGridDiscreteContinuumSolver<DIM>::Setup()
{
    if(!this->mpDensityMap)
    {
        EXCEPTION("Mesh needed before Setup can be called.");
    }

    if(this->mpDensityMap->GetGridCalculator()->HasUnstructuredGrid())
    {
        mpMesh = boost::dynamic_pointer_cast<DiscreteContinuumMesh<DIM> >(this->mpDensityMap->GetGridCalculator()->GetGrid());
        if(!mpMesh)
        {
            EXCEPTION("Can't cast to mesh during Setup");
        }
    }
    else
    {
        mpRegularGrid = boost::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpDensityMap->GetGridCalculator()->GetGrid());
        if(!mpRegularGrid)
        {
            EXCEPTION("Can't cast to regular grid during Setup");
        }
    }

    // Set up the solution
    this->mSolution = std::vector<double>(this->mpDensityMap->GetGridCalculator()->GetGrid()->GetNumberOfPoints(), 0.0);
}

template<unsigned DIM>
void AbstractMixedGridDiscreteContinuumSolver<DIM>::UpdateSolution(const std::vector<double>& data)
{
    if(this->mSolution.size()==0)
    {
        this->Setup();
    }
    this->mpDensityMap->GetGridCalculator()->GetGrid()->AddPointData(data, this->GetLabel());

    // Note, if the data vector being passed in is mPointSolution, then it will be overwritten with zeros.
    this->mSolution = std::vector<double>(data.size(), 0.0);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mSolution[i] = data[i];
    }
    this->mConcentrations = std::vector<units::quantity<unit::concentration> >(data.size(), 0.0*this->mReferenceConcentration);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mConcentrations[i] = data[i]*this->mReferenceConcentration;
    }
}

template<unsigned DIM>
void AbstractMixedGridDiscreteContinuumSolver<DIM>::UpdateElementSolution(const std::vector<double>& data)
{
    if(this->mSolution.size()==0)
    {
        this->Setup();
    }
    this->mpDensityMap->GetGridCalculator()->GetGrid()->AddCellData(data, this->GetLabel());
}

template<unsigned DIM>
void AbstractMixedGridDiscreteContinuumSolver<DIM>::UpdateSolution(const std::vector<units::quantity<unit::concentration> >& data)
{
    if(this->mSolution.size()==0)
    {
        this->Setup();
    }

    // Note, if the data vector being passed in is mPointSolution, then it will be overwritten with zeros.
    this->mSolution = std::vector<double>(data.size(), 0.0);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mSolution[i] = data[i]/this->mReferenceConcentration;
    }

    this->mpDensityMap->GetGridCalculator()->GetGrid()->AddPointData(this->mSolution, this->GetLabel());

    // Note, if the data vector being passed in is mPointSolution, then it will be overwritten with zeros.
    this->mConcentrations = std::vector<units::quantity<unit::concentration> >(data.size(), 0.0*this->mReferenceConcentration);
    for (unsigned i = 0; i < data.size(); i++)
    {
        this->mConcentrations[i] = data[i];
    }
}

template<unsigned DIM>
void AbstractMixedGridDiscreteContinuumSolver<DIM>::UpdateCellData()
{
    if(this->mSolution.size()==0)
    {
        this->Setup();
    }
    std::vector<std::vector<CellPtr> > point_cell_map = this->mpDensityMap->GetGridCalculator()->rGetCellMap();
    for(unsigned idx=0; idx<point_cell_map.size(); idx++)
    {
        for(unsigned jdx=0; jdx<point_cell_map[idx].size(); jdx++)
        {
            point_cell_map[idx][jdx]->GetCellData()->SetItem(this->mLabel, this->mSolution[idx]);
        }
    }
}

template<unsigned DIM>
void AbstractMixedGridDiscreteContinuumSolver<DIM>::Update()
{

}

template<unsigned DIM>
void AbstractMixedGridDiscreteContinuumSolver<DIM>::Write()
{
    if(this->mSolution.size()==0)
    {
        this->Setup();
    }

    if(!this->mpOutputFileHandler)
    {
        EXCEPTION("An output file handler has not been set for the DiscreteContinuum solver.");
    }

    if(PetscTools::AmMaster())
    {
        if(this->mpDensityMap->GetGridCalculator()->HasUnstructuredGrid())
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
                p_writer1->SetInput(vtkUnstructuredGrid::SafeDownCast(this->GetVtkSolution()));
            #else
                p_writer1->SetInputData(vtkUnstructuredGrid::SafeDownCast(this->GetVtkSolution()));
            #endif
            p_writer1->Update();
            p_writer1->Write();
        }
        else
        {
            RegularGridWriter writer;
            std::string extension = ".vti";
            if(PetscTools::IsParallel())
            {
                extension = ".pvti";
            }

            if(!this->mFilename.empty())
            {
                writer.SetFilename((this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "/" + this->mFilename+extension));
            }
            else
            {
                writer.SetFilename((this->mpOutputFileHandler->GetOutputDirectoryFullPath() + "/solution" + extension));
            }

            boost::shared_ptr<RegularGrid<DIM> >pRegularGrid = boost::dynamic_pointer_cast<RegularGrid<DIM> >(this->mpDensityMap->GetGridCalculator()->GetGrid());
            if(!pRegularGrid)
            {
                EXCEPTION("Can't cast to regular grid during Write");
            }

            std::vector<unsigned> whole_extents(6, 0);
            c_vector<unsigned, 3> dimensions = pRegularGrid->GetDimensions();
            whole_extents[1] = dimensions[0]-1;
            whole_extents[3] = dimensions[1]-1;
            whole_extents[5] = dimensions[2]-1;
            writer.SetWholeExtents(whole_extents);
            writer.SetImage(vtkImageData::SafeDownCast(this->GetVtkSolution()));
            writer.Write();
        }
    }
}

// Explicit instantiation
template class AbstractMixedGridDiscreteContinuumSolver<2> ;
template class AbstractMixedGridDiscreteContinuumSolver<3> ;
