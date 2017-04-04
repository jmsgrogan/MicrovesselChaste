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

#include <math.h>
#include "SimpleLinearEllipticSolver.hpp"
#include "SimpleLinearParabolicSolver.hpp"
#include "SimpleNonlinearEllipticSolver.hpp"
#include "SimpleNewtonNonlinearSolver.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "AbstractDiscreteContinuumParabolicPde.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"
#include "AbstractFiniteElementSolverBase.hpp"
#include "AbstractDiscreteContinuumPde.hpp"
#include "Exception.hpp"

template<unsigned DIM>
AbstractFiniteElementSolverBase<DIM>::AbstractFiniteElementSolverBase()
    : AbstractUnstructuredGridDiscreteContinuumSolver<DIM>(),
      mUseNewton(false),
      mGuess()
{

}

template<unsigned DIM>
AbstractFiniteElementSolverBase<DIM>::~AbstractFiniteElementSolverBase()
{

}

template <unsigned DIM>
boost::shared_ptr<AbstractFiniteElementSolverBase<DIM> > AbstractFiniteElementSolverBase<DIM>::Create()
{
    MAKE_PTR(AbstractFiniteElementSolverBase<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
void AbstractFiniteElementSolverBase<DIM>::Setup()
{
    AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::Setup();

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
}

template<unsigned DIM>
void AbstractFiniteElementSolverBase<DIM>::Update()
{
    if(this->mpPde)
    {
        this->mpPde->UpdateDiscreteSourceStrengths();
    }
    else
    {
        EXCEPTION("A PDE has not been set in the finite element solver");
    }
}

template<unsigned DIM>
void AbstractFiniteElementSolverBase<DIM>::SetGuess(const std::vector<double>& guess)
{
    mGuess = guess;
}

template<unsigned DIM>
void AbstractFiniteElementSolverBase<DIM>::SetUseSimpleNetonSolver(bool useNewton)
{
    mUseNewton = useNewton;
}

template<unsigned DIM>
void AbstractFiniteElementSolverBase<DIM>::Solve()
{
    if(!this->IsSetupForSolve)
    {
        this->Setup();
    }

    // Set up for solve
    this->mpPde->SetGridCalculator(this->mpGridCalculator);
    this->mpPde->UpdateDiscreteSourceStrengths();
}

// Explicit instantiation
template class AbstractFiniteElementSolverBase<2> ;
template class AbstractFiniteElementSolverBase<3> ;
