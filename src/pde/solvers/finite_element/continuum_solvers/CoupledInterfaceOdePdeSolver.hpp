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

#ifndef COUPLEDINTERFACEODEPDESOLVER_HPP
#define COUPLEDINTERFACEODEPDESOLVER_HPP

#include "AbstractFeVolumeIntegralAssembler.hpp"
#include "AbstractDynamicLinearPdeSolver.hpp"
#include "MassMatrixAssembler.hpp"
#include "AbstractFeSurfaceIntegralAssemblerWithMatrix.hpp"
#include "AbstractFeVolumeIntegralAssembler.hpp"
#include "BoundaryConditionsContainer.hpp"
#include "PdeSimulationTime.hpp"
#include "AbstractLinearParabolicPde.hpp"
#include "NullSurfaceIntegralCalculator.hpp"
#include "CoupledOdePdeParabolicTermAssembler.hpp"
#include "RobinConditionsSurfaceTermAssembler.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"

/**
 * A Parabolic PDE with a coupled ODE system on prescribed boundary nodes
 */
template<unsigned DIM>
class CoupledInterfaceOdePdeSolver : public AbstractDynamicLinearPdeSolver<DIM,DIM,1>
{

protected:

    /**
     * The PDE to be solved
     */
    AbstractLinearParabolicPde<DIM,DIM>* mpParabolicPde;

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

    /**
     * The constuctor will take in a mesh and a BCC, the latter will be stored as a member variable
     */
    BoundaryConditionsContainer<DIM,DIM,1>* mpBoundaryConditions;

    /**
     * Dimensionless permeability
     */
    double mPermeability;

    /**
     * Dimensionless solution in the lumped compartment
     */
    double mCurrentLumpedSolution;

    QLength mReferenceLengthScale;

    bool mUseCoupling;

    /**
     * This is the main method which needs to be implemented. It takes in the current solution, and a
     * boolean saying whether the matrix (ie A in Ax=b) is being computed or not.
     */
    void SetupLinearSystem(Vec currentSolution, bool computeMatrix);

public:
    /**
     * The constructor needs to call the parent constructor, save the BCC, ''say that the (LHS) matrix is constant
     * in time'' (so it is only computed once), and allocate memory for the RHS matrix.
     */
    CoupledInterfaceOdePdeSolver(TetrahedralMesh<DIM,DIM>* pMesh,
                               BoundaryConditionsContainer<DIM,DIM,1>* pBoundaryConditions,
                               AbstractLinearParabolicPde<DIM,DIM>* pPde);

    virtual ~CoupledInterfaceOdePdeSolver();

    /**
     * Over-ridden method to
     *
     * @param currentSolution The current solution (solution of the linear system solve)
     */
    virtual void FollowingSolveLinearSystem(Vec currentSolution);

    /**
     * Whether to store intermediate solutions, useful for debugging. Default (off).
     */
    void SetStoreIntermediateSolutions(bool store, unsigned frequency=1);

    void SetDimensionlessPermeability(double permeability);

    void SetReferenceLengthScale(QLength referenceLengthScale);

    void SetInitialDimensionlessLumpedSolution(double solution);

    void SetUseCoupling(bool useCoupling);

    double GetDimensionlessLumpedSolution();

    /**
     * Return the intermediate solutions
     */
    const std::vector<std::pair<std::vector<double>, double> >& rGetIntermediateSolutions();

};

#endif /* COUPLEDINTERFACEODEPDESOLVER */
