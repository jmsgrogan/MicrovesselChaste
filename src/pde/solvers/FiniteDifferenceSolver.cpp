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

#include <petscts.h>
#include <petscdmda.h>
#include "LinearSystem.hpp"
#include "ReplicatableVector.hpp"
#include "VesselSegment.hpp"
#include "FiniteDifferenceSolver.hpp"
#include "LinearSteadyStateDiffusionReactionPde.hpp"
#include "SimplePetscNonlinearSolver.hpp"
#include "BaseUnits.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"

// Nonlinear solve method interfaces, needed later.
template<unsigned DIM>
PetscErrorCode HyrbidFiniteDifference_ComputeResidual(SNES snes, Vec solution_guess, Vec residual, void* pContext);
#if ( PETSC_VERSION_MAJOR==3 && PETSC_VERSION_MINOR>=5 )
template<unsigned DIM>
PetscErrorCode HyrbidFiniteDifference_ComputeJacobian(SNES snes, Vec input, Mat jacobian, Mat preconditioner, void* pContext);
#else
template<unsigned DIM>
PetscErrorCode HyrbidFiniteDifference_ComputeJacobian(SNES snes,Vec input,Mat* pJacobian ,Mat* pPreconditioner,MatStructure* pMatStructure ,void* pContext);
#endif

// Parabolic solve method interfaces, needed later.
template<unsigned DIM>
PetscErrorCode ParabolicFiniteDifferenceSolver_RHSFunction(TS ts, PetscReal t, Vec currentSolution, Vec dUdt, void* pContext);

#if ( PETSC_VERSION_MAJOR==3 && PETSC_VERSION_MINOR>=5 )
template<unsigned DIM>
PetscErrorCode ParabolicFiniteDifferenceSolver_ComputeJacobian(TS ts, PetscReal t, Vec currentSolution, Mat pGlobalJacobian,
                                                               Mat pPreconditioner, void *pContext);
#else
template<unsigned DIM>
PetscErrorCode ParabolicFiniteDifferenceSolver_ComputeJacobian(TS ts, PetscReal t, Vec currentSolution ,Mat* pJacobian ,
        Mat* pPreconditioner, MatStructure* pMatStructure ,void* pContext);
#endif

template<unsigned DIM>
FiniteDifferenceSolver<DIM>::FiniteDifferenceSolver()
    :   AbstractRegularGridDiscreteContinuumSolver<DIM>(),
        mpBoundaryConditions(),
        mUpdateBoundaryConditionsEachSolve(true),
        mBoundaryConditionsSet(false),
        mParabolicSolverTimeIncrement(0.05)
{

}

template<unsigned DIM>
boost::shared_ptr<FiniteDifferenceSolver<DIM> > FiniteDifferenceSolver<DIM>::Create()
{
    MAKE_PTR(FiniteDifferenceSolver<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
FiniteDifferenceSolver<DIM>::~FiniteDifferenceSolver()
{

}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::SetParabolicSolverTimeIncrement(double timeIncrement)
{
    mParabolicSolverTimeIncrement = timeIncrement;
}

template<unsigned DIM>
boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > > FiniteDifferenceSolver<DIM>::GetRGBoundaryConditions()
{
    return mpBoundaryConditions;
}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::UpdateBoundaryConditionsEachSolve(bool doUpdate)
{
    mUpdateBoundaryConditionsEachSolve = doUpdate;
}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::Setup()
{
    // Set up the grid and PDE
    if(!this->mpRegularGrid)
    {
        EXCEPTION("This solver needs a regular grid to be set before calling Setup.");
    }

    if(!this->mpPde and !this->mpNonLinearPde and !this->mpParabolicPde)
    {
        EXCEPTION("This solver needs a PDE to be set before calling Setup.");
    }

    if(this->CellPopulationIsSet())
    {
        this->mpRegularGrid->SetCellPopulation(*(this->mpCellPopulation), this->mCellPopulationReferenceLength);
    }

    if(this->mpNetwork)
    {
    	this->mpRegularGrid->SetVesselNetwork(this->mpNetwork);
    }

    if(this->mpPde)
    {
        this->mpPde->SetRegularGrid(this->mpRegularGrid);
    }
    else if(this->mpNonLinearPde)
    {
        this->mpNonLinearPde->SetRegularGrid(this->mpRegularGrid);
    }
    else if(this->mpParabolicPde)
    {
        this->mpParabolicPde->SetRegularGrid(this->mpRegularGrid);
    }

    // Set up the boundary conditions. Use a different description from normal DiscreteContinuum BCs for efficiency.
    mpBoundaryConditions = boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > > (new std::vector<std::pair<bool, units::quantity<unit::concentration> > >(this->mpRegularGrid->GetNumberOfPoints()));
    for(unsigned idx=0; idx<this->mpRegularGrid->GetNumberOfPoints(); idx++)
    {
        (*mpBoundaryConditions)[idx] = std::pair<bool, units::quantity<unit::concentration> >(false, 0.0*this->mReferenceConcentration);
    }
    for(unsigned bound_index=0; bound_index<this->mBoundaryConditions.size(); bound_index++)
    {
        this->mBoundaryConditions[bound_index]->SetRegularGrid(this->mpRegularGrid);
    }

    // Set up the vtk solution grid
    AbstractRegularGridDiscreteContinuumSolver<DIM>::Setup();

    // Update the source strengths and boundary conditions;
    Update();

    this->IsSetupForSolve = true;
}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::Update()
{
    // Update the PDE source strengths
    if(this->mpPde)
    {
        this->mpPde->UpdateDiscreteSourceStrengths();
    }
    else if(this->mpNonLinearPde)
    {
        this->mpNonLinearPde->UpdateDiscreteSourceStrengths();
    }
    else if(this->mpParabolicPde)
    {
        this->mpParabolicPde->UpdateDiscreteSourceStrengths();
    }

    // Update the boundary conditions
    if(mUpdateBoundaryConditionsEachSolve or !mBoundaryConditionsSet)
    {
        for(unsigned bound_index=0; bound_index<this->mBoundaryConditions.size(); bound_index++)
        {
            this->mBoundaryConditions[bound_index]->UpdateRegularGridBoundaryConditions(mpBoundaryConditions);
        }
        mBoundaryConditionsSet = true;
    }
}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::DoParabolicSolve()
{
    // Set up the system
    unsigned number_of_points = this->mpRegularGrid->GetNumberOfPoints();

    // Initialize solution
    if(this->mSolution.size()==0)
    {
        this->mSolution = std::vector<double>(number_of_points, 0.0);
    }

    Vec previous_solution = PetscTools::CreateVec(number_of_points + 1);
    for(unsigned idx=0; idx<this->mSolution.size(); idx++)
    {
        PetscVecTools::SetElement(previous_solution, idx, this->mSolution[idx]);
    }
    PetscVecTools::SetElement(previous_solution, number_of_points,
            this->mpParabolicPde->GetMultiplierValue()/this->mReferenceConcentration);

    TS ts; // time stepper
    SNES snes; // nonlinear solver
    TSCreate(PETSC_COMM_WORLD,&ts);
    TSSetType(ts,TSBEULER);
    TSSetProblemType(ts,TS_NONLINEAR);

    TSSetRHSFunction(ts,NULL,ParabolicFiniteDifferenceSolver_RHSFunction<DIM>, this);

    Mat jacobian; // Jacobian Matrix
    PetscInt N; // number of elements
    // Get the size of the Jacobian from the residual
    VecGetSize(previous_solution, &N);
    // 8 is the maximum number of non-zero entries per row
    PetscTools::SetupMat(jacobian, N, N, 8, PETSC_DECIDE, PETSC_DECIDE, true, false);
    TSSetRHSJacobian(ts, jacobian, jacobian, ParabolicFiniteDifferenceSolver_ComputeJacobian<DIM>,this);

    // Time stepping and SNES settings
    PetscInt       time_steps_max = 1e7;
    PetscReal      time_total_max = SimulationTime::Instance()->GetTimeStep();
    TSGetSNES(ts,&snes);
    PetscReal abstol = 1.0e-50;
    PetscReal reltol = 1.0e-10;
    PetscReal stol = 1.0e-10;
    PetscInt maxits = 5000;
    PetscInt maxf = 100000;
    PetscInt maxFails = -1;

    SNESSetTolerances(snes, abstol, reltol, stol, maxits, maxf);
    SNESSetMaxLinearSolveFailures(snes, maxFails);
    TSSetDuration(ts, time_steps_max, time_total_max);
    TSSetExactFinalTime(ts,TS_EXACTFINALTIME_INTERPOLATE);

    // Set initial timestep
    PetscReal dt;
    dt = mParabolicSolverTimeIncrement;
    TSSetInitialTimeStep(ts, 0.0, dt);

    // Do the solve
    TSSolve(ts, previous_solution);

    // Get the current time
    PetscReal solveTime;
    TSGetSolveTime(ts, &solveTime);

    ReplicatableVector soln_repl(previous_solution);
    // Populate the solution vector
    std::vector<double> solution = std::vector<double>(number_of_points, 0.0);
    for (unsigned row = 0; row < number_of_points; row++)
    {
        solution[row] = soln_repl[row];
    }
    this->mpParabolicPde->SetMultiplierValue(soln_repl[number_of_points]*this->mReferenceConcentration);

    this->UpdateSolution(solution);

    if (this->mWriteSolution)
    {
        this->Write();
    }

    TSDestroy(&ts);
    PetscTools::Destroy(jacobian);
    PetscTools::Destroy(previous_solution);
}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::DoLinearSolve()
{
    // Set up the system
    unsigned number_of_points = this->mpRegularGrid->GetNumberOfPoints();
    unsigned extents_x = this->mpRegularGrid->GetExtents()[0];
    unsigned extents_y = this->mpRegularGrid->GetExtents()[1];
    unsigned extents_z = this->mpRegularGrid->GetExtents()[2];

    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::length> spacing = this->mpRegularGrid->GetSpacing();

    double diffusion_term = 0.0;
    if(this->mpPde)
    {
        diffusion_term = (this->mpPde->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;
    }
    else
    {
        diffusion_term = (this->mpNonLinearPde->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;
    }

    LinearSystem linear_system(number_of_points, 7);
    for (unsigned i = 0; i < extents_z; i++) // Z
    {
        for (unsigned j = 0; j < extents_y; j++) // Y
        {
            for (unsigned k = 0; k < extents_x; k++) // X
            {
                unsigned grid_index = this->mpRegularGrid->Get1dGridIndex(k, j, i);

                linear_system.AddToMatrixElement(grid_index, grid_index, this->mpPde->ComputeLinearInUCoeffInSourceTerm(grid_index)*reference_time - 6.0 * diffusion_term);

                // Assume no flux on domain boundaries by default
                // No flux at x bottom
                if (k > 0)
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index - 1, diffusion_term);
                }
                else
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at x top
                if (k < extents_x - 1)
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index + 1, diffusion_term);
                }
                else
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at y bottom
                if (j > 0)
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index - extents_x, diffusion_term);
                }
                else
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at y top
                if (j < extents_y - 1)
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index + extents_x, diffusion_term);
                }
                else
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at z bottom
                if (i > 0)
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index - extents_x * extents_y, diffusion_term);
                }
                else
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at z top
                if (i < extents_z - 1)
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index + extents_x * extents_y, diffusion_term);
                }
                else
                {
                    linear_system.AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }
                linear_system.SetRhsVectorElement(grid_index, -this->mpPde->ComputeConstantInUSourceTerm(grid_index)*(reference_time/this->mReferenceConcentration));
            }
        }
    }

    // Apply the boundary conditions
    std::vector<unsigned> bc_indices;
    for(unsigned idx=0; idx<this->mpRegularGrid->GetNumberOfPoints(); idx++)
    {
        if((*mpBoundaryConditions)[idx].first)
        {
            bc_indices.push_back(idx);
            linear_system.SetRhsVectorElement(idx, (*mpBoundaryConditions)[idx].second/this->mReferenceConcentration);
        }
    }
    linear_system.ZeroMatrixRowsWithValueOnDiagonal(bc_indices, 1.0);

    // Solve the linear system
    linear_system.AssembleFinalLinearSystem();
    ReplicatableVector soln_repl(linear_system.Solve());

    // Populate the solution vector
    std::vector<units::quantity<unit::concentration> > concs = std::vector<units::quantity<unit::concentration> >(number_of_points,
                                                                                                                  0.0*this->mReferenceConcentration);
    for (unsigned row = 0; row < number_of_points; row++)
    {
        concs[row] = soln_repl[row]*this->mReferenceConcentration;
    }

    this->UpdateSolution(concs);

    if (this->mWriteSolution)
    {
        this->Write();
    }
}

template<unsigned DIM>
void FiniteDifferenceSolver<DIM>::Solve()
{
    if(!this->IsSetupForSolve)
    {
        Setup();
    }

    if(this->mpPde)
    {
        DoLinearSolve();
    }
    else if(this->mpParabolicPde)
    {
        DoParabolicSolve();
    }
    else
    {
        // Set up initial Guess
        unsigned number_of_points = this->mpRegularGrid->GetNumberOfPoints();
        Vec initial_guess=PetscTools::CreateAndSetVec(number_of_points, 1.0);

        SimplePetscNonlinearSolver solver_petsc;
        int length = 7;
        Vec answer_petsc = solver_petsc.Solve(&HyrbidFiniteDifference_ComputeResidual<DIM>,
                                              &HyrbidFiniteDifference_ComputeJacobian<DIM>, initial_guess, length, this);

        ReplicatableVector soln_repl(answer_petsc);

        // Populate the solution vector
        this->mConcentrations = std::vector<units::quantity<unit::concentration> >(number_of_points,
                                                                                   0.0*this->mReferenceConcentration);
        for (unsigned row = 0; row < number_of_points; row++)
        {
           this->mConcentrations[row] = soln_repl[row]*this->mReferenceConcentration;
        }

        this->UpdateSolution(this->mConcentrations);

        if (this->mWriteSolution)
        {
            this->Write();
        }
    }
}

template<unsigned DIM>
PetscErrorCode HyrbidFiniteDifference_ComputeResidual(SNES snes, Vec solution_guess, Vec residual, void* pContext)
{
    FiniteDifferenceSolver<DIM>* solver = (FiniteDifferenceSolver<DIM>*) pContext;

    unsigned extents_x = solver->GetGrid()->GetExtents()[0];
    unsigned extents_y = solver->GetGrid()->GetExtents()[1];
    unsigned extents_z = solver->GetGrid()->GetExtents()[2];

    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::length> spacing = solver->GetGrid()->GetSpacing();
    double diffusion_term = (solver->GetNonLinearPde()->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;

    // It used to be possible to work directly with the solution_guess Vec, but now it seems to give read only errors.
    // Copy the vector for now.
    ReplicatableVector soln_guess_repl(solution_guess);

    // Get the residual vector
    PetscVecTools::Zero(residual);
    for (unsigned i = 0; i < extents_z; i++) // Z
    {
        for (unsigned j = 0; j < extents_y; j++) // Y
        {
            for (unsigned k = 0; k < extents_x; k++) // X
            {
                unsigned grid_index = solver->GetGrid()->Get1dGridIndex(k, j, i);
                double grid_guess = soln_guess_repl[grid_index];
                units::quantity<unit::concentration> scale_grid_guess = grid_guess*solver->GetReferenceConcentration();

                PetscVecTools::AddToElement(residual, grid_index, grid_guess * (- 6.0 * diffusion_term) +
                                                solver->GetNonLinearPde()->ComputeNonlinearSourceTerm(grid_index, scale_grid_guess)*(reference_time/solver->GetReferenceConcentration()));

                // Assume no flux on domain boundaries by default
                // No flux at x bottom
                if (k > 0)
                {
                    double neighbour_guess = soln_guess_repl[grid_index-1];
                    PetscVecTools::AddToElement(residual, grid_index, neighbour_guess * diffusion_term);
                }
                else
                {
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term * grid_guess);
                }

                // No flux at x top
                if (k < extents_x - 1)
                {
                    double neighbour_guess = soln_guess_repl[grid_index+1];
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term * neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term * grid_guess);
                }

                // No flux at y bottom
                if (j > 0)
                {
                    double neighbour_guess = soln_guess_repl[grid_index-extents_x];
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term* neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term * grid_guess);
                }

                // No flux at y top
                if (j < extents_y - 1)
                {
                    double neighbour_guess = soln_guess_repl[grid_index+extents_x];
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term* neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term * grid_guess);
                }

                // No flux at z bottom
                if (i > 0)
                {
                    double neighbour_guess = soln_guess_repl[grid_index - extents_x * extents_y];
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term*neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term * grid_guess);
                }

                // No flux at z top
                if (i < extents_z - 1)
                {
                    double neighbour_guess = soln_guess_repl[grid_index + extents_x * extents_y];
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term * neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(residual, grid_index, diffusion_term * grid_guess);
                }
            }
        }
    }

    PetscVecTools::Finalise(residual);

    // Dirichlet Boundary conditions
    std::vector<unsigned> bc_indices;
    for(unsigned idx=0; idx<solver->GetGrid()->GetNumberOfPoints(); idx++)
    {
        if((*(solver->GetRGBoundaryConditions()))[idx].first)
        {
            PetscVecTools::SetElement(residual, idx, 0.0);
        }
    }
    PetscVecTools::Finalise(residual);

    return 0;
}

#if ( PETSC_VERSION_MAJOR==3 && PETSC_VERSION_MINOR>=5 )
template<unsigned DIM>
PetscErrorCode HyrbidFiniteDifference_ComputeJacobian(SNES snes, Vec input, Mat jacobian, Mat preconditioner, void* pContext)
{
#else
template<unsigned DIM>
PetscErrorCode HyrbidFiniteDifference_ComputeJacobian(SNES snes, Vec input, Mat* pJacobian, Mat* pPreconditioner, MatStructure* pMatStructure, void* pContext)
{
    Mat jacobian = *pJacobian;
#endif
    FiniteDifferenceSolver<DIM>* solver = (FiniteDifferenceSolver<DIM>*) pContext;
    PetscMatTools::Zero(jacobian);
    PetscMatTools::SwitchWriteMode(jacobian);

    ReplicatableVector input_repl(input);

    unsigned extents_x = solver->GetGrid()->GetExtents()[0];
    unsigned extents_y = solver->GetGrid()->GetExtents()[1];
    unsigned extents_z = solver->GetGrid()->GetExtents()[2];

    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::length> spacing = solver->GetGrid()->GetSpacing();

    double diffusion_term = (solver->GetNonLinearPde()->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;

    // Get the residual vector
    for (unsigned i = 0; i < extents_z; i++) // Z
    {
        for (unsigned j = 0; j < extents_y; j++) // Y
        {
            for (unsigned k = 0; k < extents_x; k++) // X
            {
                unsigned grid_index = solver->GetGrid()->Get1dGridIndex(k, j, i);
                double grid_guess = input_repl[grid_index];
                units::quantity<unit::concentration> scale_grid_guess = grid_guess*solver->GetReferenceConcentration();

                PetscMatTools::AddToElement(jacobian, grid_index, grid_index, - 6.0 * diffusion_term +
                                            solver->GetNonLinearPde()->ComputeNonlinearSourceTermPrime(grid_index, scale_grid_guess)*reference_time);

                // Assume no flux on domain boundaries by default
                // No flux at x bottom
                if (k > 0)
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index - 1, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index, diffusion_term);
                }

                // No flux at x top
                if (k < extents_x - 1)
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index + 1, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index, diffusion_term);
                }

                // No flux at y bottom
                if (j > 0)
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index - extents_x, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index, diffusion_term);
                }

                // No flux at y top
                if (j < extents_y - 1)
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index + extents_x, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index, diffusion_term);
                }

                // No flux at z bottom
                if (i > 0)
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index - extents_x * extents_y, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(jacobian, grid_index,grid_index, diffusion_term);
                }

                // No flux at z top
                if (i < extents_z - 1)
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index + extents_x * extents_y, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(jacobian, grid_index, grid_index, diffusion_term);
                }
            }
        }
    }

    // Apply the boundary conditions
    std::vector<unsigned> bc_indices;
    for(unsigned idx=0; idx<solver->GetGrid()->GetNumberOfPoints(); idx++)
    {
        if((*(solver->GetRGBoundaryConditions()))[idx].first)
        {
            bc_indices.push_back(idx);
        }
    }

    PetscMatTools::ZeroRowsWithValueOnDiagonal(jacobian, bc_indices, 1.0);

    PetscMatTools::Finalise(jacobian);
    return 0;
}

template<unsigned DIM>
PetscErrorCode ParabolicFiniteDifferenceSolver_RHSFunction(TS ts, PetscReal t, Vec currentSolution, Vec dUdt, void* pContext)
{
    // extract solver from void so that we can use locally set values from the calculator object
    FiniteDifferenceSolver<DIM>* p_solver = (FiniteDifferenceSolver<DIM>*) pContext;
    unsigned extents_x = p_solver->GetGrid()->GetExtents()[0];
    unsigned extents_y = p_solver->GetGrid()->GetExtents()[1];
    unsigned extents_z = p_solver->GetGrid()->GetExtents()[2];
    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::length> spacing = p_solver->GetGrid()->GetSpacing();
    double diffusion_term = (p_solver->GetParabolicPde()->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;

    // compute function value, given current guess
    PetscVecTools::Zero(dUdt);

    // Check for VEGF specific PDE
    boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<DIM> > p_coupled_vegf_pde =
            boost::dynamic_pointer_cast<CoupledVegfPelletDiffusionReactionPde<DIM> >(p_solver->GetParabolicPde());

    for (unsigned i = 0; i < extents_z; i++) // Z
    {
        for (unsigned j = 0; j < extents_y; j++) // Y
        {
            for (unsigned k = 0; k < extents_x; k++) // X
            {
                unsigned grid_index = p_solver->GetGrid()->Get1dGridIndex(k, j, i);
                double current_solution = PetscVecTools::GetElement(currentSolution, grid_index);
                units::quantity<unit::concentration>  current_dimensional_solution =
                        current_solution*p_solver->GetReferenceConcentration();
                units::quantity<unit::concentration_flow_rate> sink_terms =
                        p_solver->GetParabolicPde()->ComputeNonlinearSourceTerm(grid_index, current_dimensional_solution);
                double nondim_sink_terms = sink_terms*(reference_time/p_solver->GetReferenceConcentration());
                PetscVecTools::AddToElement(dUdt, grid_index, nondim_sink_terms);
                PetscVecTools::AddToElement(dUdt, grid_index,-2.0*double(DIM)*diffusion_term*current_solution);

                // No flux at x bottom
                if (k > 0)
                {
                    double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index - 1);
                    PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                }
                else
                {
                    double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index + 1);
                    PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                }

                // No flux at x top
                if (k < extents_x - 1)
                {
                    double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index + 1);
                    PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                }
                else
                {
                    double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index - 1);
                    PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                }

                // No flux at y bottom
                if (j > 0)
                {
                    double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index - extents_x);
                    PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                }
                else
                {
                    double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index + extents_x);
                    PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                }

                // Robin at y top
                if (j < extents_y - 1)
                {
                    double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index + extents_x);
                    PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                }
                else
                {
                    double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index - extents_x);
                    PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);

                    if(p_coupled_vegf_pde)
                    {
                        double nondim_permeability =
                                p_coupled_vegf_pde->GetCorneaPelletPermeability()*(reference_time/spacing);
                        PetscVecTools::AddToElement(dUdt, grid_index, -2.0*nondim_permeability*nbr_solution);
                        double pellet_solution = PetscVecTools::GetElement(currentSolution, extents_x*extents_y*extents_z);
                        double binding_constant = p_coupled_vegf_pde->GetPelletBindingConstant();
                        PetscVecTools::AddToElement(dUdt, grid_index, 2.0*nondim_permeability*pellet_solution/binding_constant);
                    }
                }

                if(DIM>2)
                {
                    // No flux at z bottom
                    if (i > 0)
                    {
                        double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index - extents_x*extents_y);
                        PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                    }
                    else
                    {
                        double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index + extents_x*extents_y);
                        PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                    }

                    // No flux at z top
                    if (i < extents_z - 1)
                    {
                        double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index + extents_x*extents_y);
                        PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                    }
                    else
                    {
                        double nbr_solution = PetscVecTools::GetElement(currentSolution, grid_index - extents_x*extents_y);
                        PetscVecTools::AddToElement(dUdt, grid_index, diffusion_term*nbr_solution);
                    }
                }
            }
        }
    }

    if(p_coupled_vegf_pde)
    {
        // calculate d(VEGF_pellet)/dt
        double vegf_boundary_average = 0.0;
        for (unsigned i = 0; i < extents_x; i++)
        {
            for (unsigned j = 0; j < extents_z; j++)
            {
                unsigned k = i + extents_x*(extents_y - 1) + extents_x*extents_y*j;
                vegf_boundary_average += PetscVecTools::GetElement(currentSolution, k);
            }
        }
        vegf_boundary_average /= double(extents_x*extents_z);
        units::quantity<unit::area> surface_area = p_coupled_vegf_pde->GetPelletSurfaceArea();
        units::quantity<unit::volume> volume = p_coupled_vegf_pde->GetPelletVolume();
        units::quantity<unit::membrane_permeability> permeability = p_coupled_vegf_pde->GetCorneaPelletPermeability();
        units::quantity<unit::dimensionless> binding_constant = p_coupled_vegf_pde->GetPelletBindingConstant();
        units::quantity<unit::rate> decay_rate = p_coupled_vegf_pde->GetPelletFreeDecayRate();

        double pellet_solution = PetscVecTools::GetElement(currentSolution, extents_x*extents_y*extents_z);

        units::quantity<unit::rate> pellet_update_term1 = ((surface_area*permeability)/volume)*((pellet_solution/binding_constant) -
                vegf_boundary_average);
        units::quantity<unit::rate> dVegf_dt = -(decay_rate/binding_constant)*pellet_solution - pellet_update_term1;
        PetscVecTools::AddToElement(dUdt, extents_x*extents_y*extents_z, dVegf_dt*reference_time);
    }

    PetscVecTools::Finalise(dUdt);
    return 0;
}

#if ( PETSC_VERSION_MAJOR==3 && PETSC_VERSION_MINOR>=5 )
template<unsigned DIM>
PetscErrorCode ParabolicFiniteDifferenceSolver_ComputeJacobian(TS ts, PetscReal t, Vec currentSolution,
                                                               Mat pGlobalJacobian, Mat pPreconditioner, void *pContext)
{
#else
template<unsigned DIM>
PetscErrorCode ParabolicFiniteDifferenceSolver_ComputeJacobian(TS ts, PetscReal t, Vec currentSolution, Mat* pJacobian,
        Mat* pPreconditioner, MatStructure* pMatStructure, void* pContext)
{
    Mat pGlobalJacobian = *pJacobian;
#endif

    // extract solver from void so that we can use locally set values from the calculator object
    FiniteDifferenceSolver<DIM>* p_solver = (FiniteDifferenceSolver<DIM>*) pContext;
    unsigned extents_x = p_solver->GetGrid()->GetExtents()[0];
    unsigned extents_y = p_solver->GetGrid()->GetExtents()[1];
    unsigned extents_z = p_solver->GetGrid()->GetExtents()[2];
    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::length> spacing = p_solver->GetGrid()->GetSpacing();

    boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<DIM> > p_coupled_vegf_pde =
            boost::dynamic_pointer_cast<CoupledVegfPelletDiffusionReactionPde<DIM> >(p_solver->GetParabolicPde());

    double diffusion_term = (p_solver->GetParabolicPde()->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;
    PetscMatTools::Zero(pGlobalJacobian);
    PetscMatTools::SwitchWriteMode(pGlobalJacobian);

    for (unsigned i = 0; i < extents_z; i++) // Z
    {
        for (unsigned j = 0; j < extents_y; j++) // Y
        {
            for (unsigned k = 0; k < extents_x; k++) // X
            {
                unsigned grid_index = p_solver->GetGrid()->Get1dGridIndex(k, j, i);
                double current_solution = PetscVecTools::GetElement(currentSolution, grid_index);
                units::quantity<unit::concentration>  current_dimensional_solution =
                        current_solution*p_solver->GetReferenceConcentration();
                units::quantity<unit::rate> sink_terms =
                        p_solver->GetParabolicPde()->ComputeNonlinearSourceTermPrime(grid_index, current_dimensional_solution);
                double nondim_sink_terms = sink_terms*reference_time;
                PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index, nondim_sink_terms);
                PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index,-2.0*double(DIM)*diffusion_term);

                // No flux at x = 0 boundary
                if (k>0)
                {
                    PetscMatTools::AddToElement(pGlobalJacobian,grid_index, grid_index-1, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(pGlobalJacobian,grid_index, grid_index+1, diffusion_term);
                }

                // No flux at x != 0 boundary
                if (k<extents_x-1 )
                {
                    PetscMatTools::AddToElement(pGlobalJacobian,grid_index, grid_index+1, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(pGlobalJacobian,grid_index, grid_index-1, diffusion_term);
                }

                // No flux at y = 0 boundary
                if (j>0)
                {
                    PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index - extents_x, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index + extents_x, diffusion_term);
                }

                // Robin BC at y != 0 boundary
                if (j<extents_y-1)
                {
                    PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index + extents_x, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index - extents_x, diffusion_term);
                    if(p_coupled_vegf_pde)
                    {
                        double nondim_permeability =
                                p_coupled_vegf_pde->GetCorneaPelletPermeability()*(reference_time/spacing);
                        double binding_constant = p_coupled_vegf_pde->GetPelletBindingConstant();

                        PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index, -2.0*nondim_permeability);
                        PetscMatTools::AddToElement(pGlobalJacobian, grid_index, extents_x*extents_y*extents_z,
                                2.0*nondim_permeability/binding_constant);
                    }
                }

                if(DIM>2)
                {
                    // No flux at z = 0 boundary
                    if (i>0)
                    {
                        PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index - extents_x*extents_y, diffusion_term);
                    }
                    else
                    {
                        PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index + extents_x*extents_y, diffusion_term);
                    }

                    // No flux at z != 0 boundary
                    if (i<extents_z-1)
                    {
                        PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index + extents_x*extents_y, diffusion_term);
                    }
                    else
                    {
                        PetscMatTools::AddToElement(pGlobalJacobian, grid_index, grid_index - extents_x*extents_y, diffusion_term);
                    }
                }
            }
        }
    }

    if(p_coupled_vegf_pde)
    {
        units::quantity<unit::area> surface_area = p_coupled_vegf_pde->GetPelletSurfaceArea();
        units::quantity<unit::volume> volume = p_coupled_vegf_pde->GetPelletVolume();
        units::quantity<unit::membrane_permeability> permeability = p_coupled_vegf_pde->GetCorneaPelletPermeability();
        units::quantity<unit::dimensionless> binding_constant = p_coupled_vegf_pde->GetPelletBindingConstant();
        units::quantity<unit::rate> decay_rate = p_coupled_vegf_pde->GetPelletFreeDecayRate();
        units::quantity<unit::rate> jacEntry = -(decay_rate/binding_constant) - (surface_area*permeability)/(volume*binding_constant);
        PetscMatTools::AddToElement(pGlobalJacobian, extents_x*extents_y*extents_z, extents_x*extents_y*extents_z,
                jacEntry*reference_time);

        for (unsigned i = 0; i < extents_x; i++)
        {
            for (unsigned j = 0; j < extents_z; j++)
            {
                unsigned J_index = i + extents_x*(extents_y - 1) + extents_x*extents_y*j;
                units::quantity<unit::rate> jacEntry = surface_area*permeability/(volume*double(extents_x*extents_z));
                PetscMatTools::AddToElement(pGlobalJacobian,extents_x*extents_y*extents_z,
                        J_index, jacEntry*reference_time);
            }
        }
    }

    PetscMatTools::Finalise(pGlobalJacobian);
    return 0;
}

// Explicit instantiation
template class FiniteDifferenceSolver<2>;
template class FiniteDifferenceSolver<3>;
