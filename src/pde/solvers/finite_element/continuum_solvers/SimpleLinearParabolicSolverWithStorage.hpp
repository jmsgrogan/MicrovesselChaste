
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

#ifndef SIMPLELINEARPARABOLICSOLVERWITHSTORAGE_HPP_
#define SIMPLELINEARPARABOLICSOLVERWITHSTORAGE_HPP_

#include "SimpleLinearParabolicSolver.hpp"
#include "AbstractLinearParabolicPde.hpp"
#include "UnitCollection.hpp"

/**
 * SimpleLinearParabolicSolverWithStorage.
 *
 * Solver for solving AbstractLinearParabolicPdes
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
class SimpleLinearParabolicSolverWithStorage : public SimpleLinearParabolicSolver<ELEMENT_DIM, SPACE_DIM>
{

protected:

    /**
     * Storage for intermediate solutions. Useful for debugging.
     */
    std::vector<std::pair<std::vector<double>, double> > mIntermediateSolutionCollection;

    /**
     * How often to store intermediate solutions.
     */
    unsigned mIntermediateSolutionFrequency;

    /**
     * Whether to store intermediate solutions.
     */
    bool mStoreIntermediate;

    /**
     * Count the time increments taken
     */
    unsigned mIncrementCounter;

    /**
     * Save the current time
     */
    double mCurrentTime;

public:

    /**
     * Constructor.
     *
     * @param pMesh pointer to the mesh
     * @param pPde pointer to the PDE
     * @param pBoundaryConditions pointer to the boundary conditions
     */
    SimpleLinearParabolicSolverWithStorage(AbstractTetrahedralMesh<ELEMENT_DIM,SPACE_DIM>* pMesh,
                                AbstractLinearParabolicPde<ELEMENT_DIM,SPACE_DIM>* pPde,
                                BoundaryConditionsContainer<ELEMENT_DIM,SPACE_DIM,1>* pBoundaryConditions);

    /**
     * The static and dynamic Solve() implementations both call this immediately after
     * the linear solve is carried out (but before the timestep counter is incremented.
     * This can be overloaded if further work on the solution vector needs to be done
     * (for example, in operator splitting of the diffusion and reaction terms in the
     * OperatorSplittingMonodomainSolver.
     *
     * @param currentSolution The current solution (solution of the linear system solve)
     */
    void FollowingSolveLinearSystem(Vec currentSolution);

    /**
     * Whether to store intermediate solutions, useful for debugging. Default (off).
     */
    void SetStoreIntermediateSolutions(bool store, unsigned frequency=1);

    /**
     * Return the intermediate solutions
     */
    const std::vector<std::pair<std::vector<double>, double> >& rGetIntermediateSolutions();
};

#endif /*SIMPLELINEARPARABOLICSOLVERWITHSTORAGE_HPP_*/
