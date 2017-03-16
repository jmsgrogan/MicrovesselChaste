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

#include <petscmat.h>
#include "ReplicatableVector.hpp"
#include "VesselSegment.hpp"
#include "AbstractFiniteDifferenceSolverBase.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
AbstractFiniteDifferenceSolverBase<DIM>::AbstractFiniteDifferenceSolverBase()
    :   AbstractRegularGridDiscreteContinuumSolver<DIM>(),
        mpBoundaryConditions(),
        mUpdateBoundaryConditionsEachSolve(true),
        mBoundaryConditionsSet(false),
        mMatrixToAssemble(),
        mVectorToAssemble(),
        mCurrentSolution()
{

}

template<unsigned DIM>
AbstractFiniteDifferenceSolverBase<DIM>::~AbstractFiniteDifferenceSolverBase()
{

}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::AddDiscreteTermsToMatrix()
{

}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::AddDiscreteTermsToRhs()
{

}

template<unsigned DIM>
boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > > AbstractFiniteDifferenceSolverBase<DIM>::GetRGBoundaryConditions()
{
    return mpBoundaryConditions;
}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::SetMatrixToAssemble(Mat& rMatToAssemble)
{
    mMatrixToAssemble = rMatToAssemble;
}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::SetVectorToAssemble(Vec& rVecToAssemble)
{
    mVectorToAssemble = rVecToAssemble;
}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::SetCurrentSolution(const Vec& rCurrentSolution)
{
    mCurrentSolution = rCurrentSolution;
}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::UpdateBoundaryConditionsEachSolve(bool doUpdate)
{
    mUpdateBoundaryConditionsEachSolve = doUpdate;
}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::Setup()
{
    // Set up the grid and PDE
    if(!this->mpGridCalculator)
    {
        EXCEPTION("This solver needs a grid calculator to be set before calling Setup.");
    }

    if(this->CellPopulationIsSet())
    {
        this->mpGridCalculator->SetCellPopulation(*(this->mpCellPopulation), this->mCellPopulationReferenceLength);
    }

    if(this->mpNetwork)
    {
    	this->mpGridCalculator->SetVesselNetwork(this->mpNetwork);
    }

    if(this->mpPde)
    {
        this->mpPde->SetGridCalculator(this->mpGridCalculator);
    }
    else
    {
        EXCEPTION("This solver needs a PDE to be set before calling Setup.");
    }

    // Set up the boundary conditions. Use a different description from normal DiscreteContinuum BCs for efficiency.
    unsigned num_points = this->mpGridCalculator->GetGrid()->GetNumberOfLocations();
    mpBoundaryConditions = boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > > (new std::vector<std::pair<bool, units::quantity<unit::concentration> > >(num_points));
    for(unsigned idx=0; idx<num_points; idx++)
    {
        (*mpBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(false, 0.0*this->mReferenceConcentration);
    }
    for(unsigned bound_index=0; bound_index<this->mBoundaryConditions.size(); bound_index++)
    {
        this->mBoundaryConditions[bound_index]->SetGridCalculator(this->mpGridCalculator);
    }

    // Set up the vtk solution grid
    AbstractRegularGridDiscreteContinuumSolver<DIM>::Setup();
}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::Update()
{
    // Update the PDE source strengths
    if(this->mpPde)
    {
        this->mpPde->UpdateDiscreteSourceStrengths();
    }

    // Update the boundary conditions
    if(mUpdateBoundaryConditionsEachSolve or !mBoundaryConditionsSet)
    {
        for(unsigned bound_index=0; bound_index<this->mBoundaryConditions.size(); bound_index++)
        {
            this->mBoundaryConditions[bound_index]->UpdateBoundaryConditions(mpBoundaryConditions);
        }
        mBoundaryConditionsSet = true;
    }
}

template<unsigned DIM>
void AbstractFiniteDifferenceSolverBase<DIM>::Solve()
{
    if(!this->IsSetupForSolve)
    {
        Setup();
    }
    Update();
}

// Explicit instantiation
template class AbstractFiniteDifferenceSolverBase<2>;
template class AbstractFiniteDifferenceSolverBase<3>;
