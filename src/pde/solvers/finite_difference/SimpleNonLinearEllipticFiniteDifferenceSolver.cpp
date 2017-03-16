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

#include "ReplicatableVector.hpp"
#include "SimplePetscNonlinearSolver.hpp"
#include "VesselSegment.hpp"
#include "SimpleNonLinearEllipticFiniteDifferenceSolver.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"
#include "BaseUnits.hpp"

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


template<unsigned DIM>
SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>::SimpleNonLinearEllipticFiniteDifferenceSolver()
    :   AbstractFiniteDifferenceSolverBase<DIM>()
{

}

template<unsigned DIM>
SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>::~SimpleNonLinearEllipticFiniteDifferenceSolver()
{

}

template <unsigned DIM>
boost::shared_ptr<SimpleNonLinearEllipticFiniteDifferenceSolver<DIM> > SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>::Create()
{
    MAKE_PTR(SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
void SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>::ComputeJacobian(const Vec currentGuess, Mat* pJacobian)
{
    this->SetMatrixToAssemble(*pJacobian);
    this->SetCurrentSolution(currentGuess);
    this->AssembleMatrix();

    // Apply the boundary conditions
    std::vector<unsigned> bc_indices;
    unsigned lo = this->mpRegularGrid->GetDistributedVectorFactory()->GetLow();
    unsigned hi = this->mpRegularGrid->GetDistributedVectorFactory()->GetHigh();
    for(unsigned idx=lo; idx<hi; idx++)
    {
        if((*this->mpBoundaryConditions)[idx].first)
        {
            bc_indices.push_back(idx);
        }
    }

    PetscMatTools::ZeroRowsWithValueOnDiagonal(this->mMatrixToAssemble, bc_indices, 1.0);
    PetscMatTools::Finalise(this->mMatrixToAssemble);
}

template<unsigned DIM>
void SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>::ComputeResidual(const Vec currentGuess, Vec residualVector)
{
    this->SetVectorToAssemble(residualVector);
    this->SetCurrentSolution(currentGuess);
    this->AssembleVector();

    // Apply BCs
    std::vector<unsigned> bc_indices;
    unsigned lo = this->mpRegularGrid->GetDistributedVectorFactory()->GetLow();
    unsigned hi = this->mpRegularGrid->GetDistributedVectorFactory()->GetHigh();
    for(unsigned idx=lo; idx<hi; idx++)
    {
        if((*(this->mpBoundaryConditions))[idx].first)
        {
            PetscVecTools::SetElement(this->mVectorToAssemble, idx, 0.0);
        }
    }
    PetscVecTools::Finalise(this->mVectorToAssemble);
}

template<unsigned DIM>
void SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>::AssembleMatrix()
{
    PetscMatTools::Zero(this->mMatrixToAssemble);
    PetscMatTools::SwitchWriteMode(this->mMatrixToAssemble);
    ReplicatableVector input_repl(this->mCurrentSolution);

    c_vector<unsigned, 6> extents = this->mpRegularGrid->GetExtents();
    c_vector<unsigned, 3> dimensions = this->mpRegularGrid->GetDimensions();
    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::length> spacing = this->mpRegularGrid->GetSpacing();

    boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > p_nonlinear_pde =
                boost::dynamic_pointer_cast<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> >(this->GetPde());
    if(!p_nonlinear_pde)
    {
        EXCEPTION("Could not correctly cast PDE");
    }
    double diffusion_term = (p_nonlinear_pde->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;

    // Get the residual vector
    for (unsigned i = extents[4]; i <= extents[5]; i++) // Z
    {
        for (unsigned j = extents[2]; j <= extents[3]; j++) // Y
        {
            for (unsigned k = extents[0]; k <= extents[1]; k++) // X
            {
                unsigned grid_index = this->mpRegularGrid->GetGlobalGridIndex(k, j, i);
                double grid_guess = input_repl[grid_index];
                units::quantity<unit::concentration> scale_grid_guess = grid_guess*this->GetReferenceConcentration();

                PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index, -6.0 * diffusion_term +
                        p_nonlinear_pde->ComputeNonlinearSourceTermPrime(grid_index, scale_grid_guess)*reference_time);

                // Assume no flux on domain boundaries by default
                // No flux at x bottom
                if (k > 0)
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index - 1, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index, diffusion_term);
                }

                // No flux at x top
                if (k < dimensions[0] - 1)
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index + 1, diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index, diffusion_term);
                }

                // No flux at y bottom
                if (j > 0)
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index - dimensions[0], diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index, diffusion_term);
                }

                // No flux at y top
                if (j < dimensions[1] - 1)
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index + dimensions[0], diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index, diffusion_term);
                }

                // No flux at z bottom
                if (i > 0)
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index - dimensions[0] * dimensions[1], diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index,grid_index, diffusion_term);
                }

                // No flux at z top
                if (i < dimensions[2] - 1)
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index + dimensions[0] * dimensions[1], diffusion_term);
                }
                else
                {
                    PetscMatTools::AddToElement(this->mMatrixToAssemble, grid_index, grid_index, diffusion_term);
                }
            }
        }
    }
}

template<unsigned DIM>
void SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>::AssembleVector()
{
    c_vector<unsigned, 6> extents = this->mpRegularGrid->GetExtents();
    c_vector<unsigned, 3> dimensions = this->mpRegularGrid->GetDimensions();
    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::length> spacing = this->mpRegularGrid->GetSpacing();

    boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > p_nonlinear_pde =
                boost::dynamic_pointer_cast<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> >(this->mpPde);
    if(!p_nonlinear_pde)
    {
        EXCEPTION("Could not correctly cast PDE");
    }

    double diffusion_term = (p_nonlinear_pde->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;

    // It used to be possible to work directly with the solution_guess Vec, but now it seems to give read only errors.
    // Copy the vector for now.
    ReplicatableVector soln_guess_repl(this->mCurrentSolution);

    // Get the residual vector
    PetscVecTools::Zero(this->mVectorToAssemble);
    for (unsigned i = extents[4]; i <= extents[5]; i++) // Z
    {
        for (unsigned j = extents[2]; j <= extents[3]; j++) // Y
        {
            for (unsigned k = extents[0]; k <= extents[1]; k++) // X
            {
                unsigned grid_index = this->mpRegularGrid->GetGlobalGridIndex(k, j, i);
                double grid_guess = soln_guess_repl[grid_index];
                units::quantity<unit::concentration> scale_grid_guess = grid_guess*this->GetReferenceConcentration();

                PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, grid_guess * (- 6.0 * diffusion_term) +
                        p_nonlinear_pde->ComputeNonlinearSourceTerm(grid_index, scale_grid_guess)*(reference_time/this->GetReferenceConcentration()));

                // Assume no flux on domain boundaries by default
                // No flux at x bottom
                if (k > 0)
                {
                    double neighbour_guess = soln_guess_repl[grid_index-1];
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, neighbour_guess * diffusion_term);
                }
                else
                {
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term * grid_guess);
                }

                // No flux at x top
                if (k < dimensions[0] - 1)
                {
                    double neighbour_guess = soln_guess_repl[grid_index+1];
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term * neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term * grid_guess);
                }

                // No flux at y bottom
                if (j > 0)
                {
                    double neighbour_guess = soln_guess_repl[grid_index-dimensions[0]];
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term* neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term * grid_guess);
                }

                // No flux at y top
                if (j < dimensions[1] - 1)
                {
                    double neighbour_guess = soln_guess_repl[grid_index+dimensions[0]];
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term* neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term * grid_guess);
                }

                // No flux at z bottom
                if (i > 0)
                {
                    double neighbour_guess = soln_guess_repl[grid_index - dimensions[0] * dimensions[1]];
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term*neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term * grid_guess);
                }

                // No flux at z top
                if (i < dimensions[2] - 1)
                {
                    double neighbour_guess = soln_guess_repl[grid_index + dimensions[0] * dimensions[1]];
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term * neighbour_guess);
                }
                else
                {
                    PetscVecTools::AddToElement(this->mVectorToAssemble, grid_index, diffusion_term * grid_guess);
                }
            }
        }
    }

    PetscVecTools::Finalise(this->mVectorToAssemble);
}

template<unsigned DIM>
void SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>::Solve()
{
    if(!this->IsSetupForSolve)
    {
        this->Setup();
    }
    this->Update();

    // Set up initial Guess
    c_vector<unsigned, 3> dimensions = this->mpRegularGrid->GetDimensions();
    unsigned number_of_points = dimensions[0]*dimensions[1]*dimensions[2];
    Vec initial_guess = PetscTools::CreateAndSetVec(number_of_points, 1.0);

    int length = 7;
    SimplePetscNonlinearSolver solver_petsc;
    Vec answer_petsc = solver_petsc.Solve(&HyrbidFiniteDifference_ComputeResidual<DIM>,
                                          &HyrbidFiniteDifference_ComputeJacobian<DIM>, initial_guess, length, this);
    ReplicatableVector soln_repl(answer_petsc);

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
PetscErrorCode HyrbidFiniteDifference_ComputeResidual(SNES snes, Vec solution_guess, Vec residual, void* pContext)
{
    SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>* p_solver =
            (SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>*) pContext;
    p_solver->ComputeResidual(solution_guess, residual);
    return 0;
}

#if ( PETSC_VERSION_MAJOR==3 && PETSC_VERSION_MINOR>=5 )
template<unsigned DIM>
PetscErrorCode HyrbidFiniteDifference_ComputeJacobian(SNES snes, Vec input, Mat jacobian, Mat preconditioner, void* pContext)
{
    SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>* p_solver =
            (SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>*) pContext;
    p_solver->ComputeJacobian(input, &jacobian);
#else
template<unsigned DIM>
PetscErrorCode HyrbidFiniteDifference_ComputeJacobian(SNES snes, Vec input, Mat* pJacobian, Mat* pPreconditioner, MatStructure* pMatStructure, void* pContext)
{
    SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>* p_solver =
            (SimpleNonLinearEllipticFiniteDifferenceSolver<DIM>*) pContext;
    p_solver->ComputeJacobian(input, pJacobian);
#endif
    return 0;
}

// Explicit instantiation
template class SimpleNonLinearEllipticFiniteDifferenceSolver<2>;
template class SimpleNonLinearEllipticFiniteDifferenceSolver<3>;
