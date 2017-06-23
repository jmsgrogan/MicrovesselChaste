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
std::shared_ptr<AbstractFiniteElementSolverBase<DIM> > AbstractFiniteElementSolverBase<DIM>::Create()
{
    return std::make_shared<AbstractFiniteElementSolverBase<DIM> >();

}

template<unsigned DIM>
void AbstractFiniteElementSolverBase<DIM>::Setup()
{
    AbstractUnstructuredGridDiscreteContinuumSolver<DIM>::Setup();

    if(!this->mpPde)
    {
        EXCEPTION("This solver needs a PDE to be set before calling Setup.");
    }

    std::vector<std::shared_ptr<DiscreteSource<DIM> > > discrete_sources = this->mpPde->GetDiscreteSources();
    for(unsigned idx=0;idx<discrete_sources.size();idx++)
    {
        discrete_sources[idx]->SetDensityMap(this->mpDensityMap);
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
    this->mpPde->UpdateDiscreteSourceStrengths();
}

// Explicit instantiation
template class AbstractFiniteElementSolverBase<2> ;
template class AbstractFiniteElementSolverBase<3> ;
