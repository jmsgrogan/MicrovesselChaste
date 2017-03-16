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

#include "SimpleLinearParabolicSolverWithStorage.hpp"
#include "ReplicatableVector.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
SimpleLinearParabolicSolverWithStorage<ELEMENT_DIM,SPACE_DIM>::SimpleLinearParabolicSolverWithStorage(
                            AbstractTetrahedralMesh<ELEMENT_DIM,SPACE_DIM>* pMesh,
                            AbstractLinearParabolicPde<ELEMENT_DIM,SPACE_DIM>* pPde,
                            BoundaryConditionsContainer<ELEMENT_DIM,SPACE_DIM,1>* pBoundaryConditions)
    : SimpleLinearParabolicSolver<ELEMENT_DIM,SPACE_DIM>(pMesh,pPde, pBoundaryConditions),
      mIntermediateSolutionCollection(),
      mIntermediateSolutionFrequency(1),
      mStoreIntermediate(false),
      mIncrementCounter(0),
      mCurrentTime(0.0)
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SimpleLinearParabolicSolverWithStorage<ELEMENT_DIM,SPACE_DIM>::SetStoreIntermediateSolutions(bool store,
        unsigned frequency)
{
    mStoreIntermediate = store;
    mIntermediateSolutionFrequency = frequency;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
const std::vector<std::pair<std::vector<double>, double> >& SimpleLinearParabolicSolverWithStorage<ELEMENT_DIM,SPACE_DIM>::rGetIntermediateSolutions()
{
    return mIntermediateSolutionCollection;
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
void SimpleLinearParabolicSolverWithStorage<ELEMENT_DIM,SPACE_DIM>::FollowingSolveLinearSystem(Vec currentSolution)
{
    mCurrentTime += this->mLastWorkingTimeStep;
    mIncrementCounter+=1;

    if(mStoreIntermediate)
    {
        if(mIncrementCounter%mIntermediateSolutionFrequency==0)
        {
            ReplicatableVector solution_repl(currentSolution);
            std::vector<double> solution = std::vector<double>(solution_repl.GetSize());
            for(unsigned idx = 0; idx < solution_repl.GetSize(); idx++)
            {
                solution[idx] = solution_repl[idx];
            }
            mIntermediateSolutionCollection.push_back(std::pair<std::vector<double>, double>(solution, mCurrentTime));
        }
    }
}

// Explicit instantiation
template class SimpleLinearParabolicSolverWithStorage<1,1>;
template class SimpleLinearParabolicSolverWithStorage<2,2>;
template class SimpleLinearParabolicSolverWithStorage<3,3>;
