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
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkGradientFilter.h>
#include <vtkDataArray.h>
#include "AbstractDiscreteContinuumPde.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
AbstractDiscreteContinuumSolver<DIM>::AbstractDiscreteContinuumSolver()
    :   mpOutputFileHandler(),
        mFilename(),
        mLabel("Default"),
        IsSetupForSolve(false),
        mWriteSolution(false),
        mpPde(),
        mBoundaryConditions(),
        mReferenceConcentration(BaseUnits::Instance()->GetReferenceConcentrationScale()),
        mSolution(),
        mConcentrations(),
        mpDensityMap(DensityMap<DIM>::Create())
{

}

template<unsigned DIM>
AbstractDiscreteContinuumSolver<DIM>::~AbstractDiscreteContinuumSolver()
{

}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::AddBoundaryCondition(boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > pBoundaryCondition)
{
    mBoundaryConditions.push_back(pBoundaryCondition);
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractDiscreteContinuumSolver<DIM>::GetConcentrations()
{
    return mConcentrations;
}

template<unsigned DIM>
const std::string& AbstractDiscreteContinuumSolver<DIM>::GetLabel()
{
    return mLabel;
}

template<unsigned DIM>
boost::shared_ptr<DensityMap<DIM> > AbstractDiscreteContinuumSolver<DIM>::GetDensityMap()
{
    return mpDensityMap;
}

template<unsigned DIM>
boost::shared_ptr<AbstractDiscreteContinuumPde<DIM, DIM> > AbstractDiscreteContinuumSolver<DIM>::GetPde()
{
    if(!mpPde)
    {
        EXCEPTION("A pde has not been set.");
    }
    return mpPde;
}

template<unsigned DIM>
units::quantity<unit::length> AbstractDiscreteContinuumSolver<DIM>::GetReferenceLength()
{
    return mpDensityMap->GetGridCalculator()->GetGrid()->GetReferenceLengthScale();
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractDiscreteContinuumSolver<DIM>::GetConcentrations(vtkSmartPointer<vtkPoints> pSamplePoints)
{
    std::vector<units::quantity<unit::concentration> > sampled_solution(pSamplePoints->GetNumberOfPoints(),
            0.0*this->mReferenceConcentration);


    // Sample the field at these locations. comment.
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    p_polydata->SetPoints(pSamplePoints);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_probe_filter->SetInput(p_polydata);
        p_probe_filter->SetSource(GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid());
    #else
        p_probe_filter->SetInputData(p_polydata);
        p_probe_filter->SetSourceData(GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid());
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
std::vector<units::quantity<unit::concentration> > AbstractDiscreteContinuumSolver<DIM>::GetConcentrations(boost::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid)
{
    return this->GetConcentrations(pGrid->GetPoints());
}

template<unsigned DIM>
std::vector<double> AbstractDiscreteContinuumSolver<DIM>::GetSolution(vtkSmartPointer<vtkPoints> pSamplePoints)
{
    std::vector<double> sampled_solution(pSamplePoints->GetNumberOfPoints(), 0.0);

    // Sample the field at these locations
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    p_polydata->SetPoints(pSamplePoints);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_probe_filter->SetInput(p_polydata);
        p_probe_filter->SetSource(GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid());
    #else
        p_probe_filter->SetInputData(p_polydata);
        p_probe_filter->SetSourceData(GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid());
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
std::vector<c_vector<double, 3 > > AbstractDiscreteContinuumSolver<DIM>::GetSolutionGradients(vtkSmartPointer<vtkPoints> pSamplePoints)
{
    std::vector<c_vector<double, 3 > > sampled_solution(pSamplePoints->GetNumberOfPoints(), zero_vector<double>(3));

    // Sample the field at these locations
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    p_polydata->SetPoints(pSamplePoints);

    vtkSmartPointer<vtkGradientFilter> p_gradient = vtkSmartPointer<vtkGradientFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_gradient->SetInput(GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid());
    #else
        p_gradient->SetInputData(GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid());
    #endif
    p_gradient->SetResultArrayName("Gradients");
    p_gradient->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, this->mLabel.c_str());
    p_gradient->Update();

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_probe_filter->SetInput(p_polydata);
        p_probe_filter->SetSource(p_gradient->GetOutput());
    #else
        p_probe_filter->SetInputData(p_polydata);
        p_probe_filter->SetSourceData(p_gradient->GetOutput());
    #endif
    p_probe_filter->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "Gradients");
    p_probe_filter->Update();
    vtkDataArray* p_results = p_probe_filter->GetOutput()->GetPointData()->GetArray("Gradients");
    unsigned num_points = p_results->GetNumberOfTuples();
    for(unsigned idx=0; idx<num_points; idx++)
    {
        c_vector<double, 3> result;
        p_results->GetTuple(idx, &result[0]);
        sampled_solution[idx] = result;
    }
    return sampled_solution;
}

template<unsigned DIM>
std::vector<double> AbstractDiscreteContinuumSolver<DIM>::GetSolutionP(vtkPoints* pSamplePoints)
{
    std::vector<double> sampled_solution(pSamplePoints->GetNumberOfPoints(), 0.0);

    // Sample the field at these locations
    vtkSmartPointer<vtkPolyData> p_polydata = vtkSmartPointer<vtkPolyData>::New();
    p_polydata->SetPoints(pSamplePoints);

    vtkSmartPointer<vtkProbeFilter> p_probe_filter = vtkSmartPointer<vtkProbeFilter>::New();
    #if VTK_MAJOR_VERSION <= 5
        p_probe_filter->SetInput(p_polydata);
        p_probe_filter->SetSource(GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid());
    #else
        p_probe_filter->SetInputData(p_polydata);
        p_probe_filter->SetSourceData(GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid());
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
std::vector<double> AbstractDiscreteContinuumSolver<DIM>::GetSolution(boost::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid)
{
    return this->GetSolution(pGrid->GetPoints());
}

template<unsigned DIM>
units::quantity<unit::concentration> AbstractDiscreteContinuumSolver<DIM>::GetReferenceConcentration()
{
    return mReferenceConcentration;
}

template<unsigned DIM>
std::vector<double> AbstractDiscreteContinuumSolver<DIM>::GetSolution()
{
    return mSolution;
}

template<unsigned DIM>
vtkSmartPointer<vtkDataSet> AbstractDiscreteContinuumSolver<DIM>::GetVtkSolution()
{
    return GetDensityMap()->GetGridCalculator()->GetGrid()->GetVtkGrid();
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetFileHandler(boost::shared_ptr<OutputFileHandler> pOutputFileHandler)
{
    mpOutputFileHandler = pOutputFileHandler;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetFileName(const std::string& rFilename)
{
    mFilename = rFilename;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetLabel(const std::string& rLabel)
{
    mLabel = rLabel;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetGrid(boost::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid)
{
    mpDensityMap->SetGrid(pGrid);
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetDensityMap(boost::shared_ptr<DensityMap<DIM> > pDensityMap)
{
    mpDensityMap = pDensityMap;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetPde(boost::shared_ptr<AbstractDiscreteContinuumPde<DIM, DIM> > pPde)
{
    mpPde = pPde;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetReferenceConcentration(units::quantity<unit::concentration> referenceConcentration)
{
    mReferenceConcentration = referenceConcentration;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation,
                                                             units::quantity<unit::length> cellPopulationReferenceLength,
                                                             units::quantity<unit::concentration> cellPopulationReferenceConcentration)
{
    mpDensityMap->SetCellPopulation(rCellPopulation, cellPopulationReferenceLength, cellPopulationReferenceConcentration);
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpDensityMap->SetVesselNetwork(pNetwork);
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetWriteSolution(bool write)
{
    mWriteSolution = write;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::UpdateSolution(const std::vector<double>& data)
{
    mSolution = data;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::UpdateSolution(const std::vector<units::quantity<unit::concentration> >& data)
{
    mConcentrations = data;
}

// Explicit instantiation
template class AbstractDiscreteContinuumSolver<2> ;
template class AbstractDiscreteContinuumSolver<3> ;
