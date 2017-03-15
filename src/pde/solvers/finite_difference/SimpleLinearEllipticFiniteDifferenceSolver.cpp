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

#include <petscts.h>
#include <petscdmda.h>
#include "ReplicatableVector.hpp"
#include "VesselSegment.hpp"
#include "SimpleLinearEllipticFiniteDifferenceSolver.hpp"
#include "DiscreteContinuumLinearEllipticPde.hpp"
#include "BaseUnits.hpp"
#include "Debug.hpp"

template<unsigned DIM>
SimpleLinearEllipticFiniteDifferenceSolver<DIM>::SimpleLinearEllipticFiniteDifferenceSolver()
    :   AbstractFiniteDifferenceSolverBase<DIM>(),
        mpLinearSystem(),
        mpInitialLhs(),
        mpInitialRhs()
{

}

template<unsigned DIM>
SimpleLinearEllipticFiniteDifferenceSolver<DIM>::~SimpleLinearEllipticFiniteDifferenceSolver()
{

}

template <unsigned DIM>
boost::shared_ptr<SimpleLinearEllipticFiniteDifferenceSolver<DIM> > SimpleLinearEllipticFiniteDifferenceSolver<DIM>::Create()
{
    MAKE_PTR(SimpleLinearEllipticFiniteDifferenceSolver<DIM>, pSelf);
    return pSelf;
}

template<unsigned DIM>
boost::shared_ptr<LinearSystem> SimpleLinearEllipticFiniteDifferenceSolver<DIM>::GetLinearSystem()
{
    return mpLinearSystem;
}

template<unsigned DIM>
void SimpleLinearEllipticFiniteDifferenceSolver<DIM>::AddDiscreteTermsToMatrix()
{
    boost::shared_ptr<DiscreteContinuumLinearEllipticPde<DIM, DIM> > p_linear_pde =
                boost::dynamic_pointer_cast<DiscreteContinuumLinearEllipticPde<DIM, DIM> >(this->mpPde);
    if(!p_linear_pde)
    {
        EXCEPTION("PDE not recognized");
    }

    this->mpLinearSystem->SwitchWriteModeLhsMatrix();
    c_vector<unsigned, 6> extents = this->mpRegularGrid->GetExtents();
    c_vector<unsigned, 3> dimensions = this->mpRegularGrid->GetDimensions();
    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    for (unsigned i = extents[4]; i <= extents[5]; i++) // Z
    {
        for (unsigned j = extents[2]; j <= extents[3]; j++) // Y
        {
            for (unsigned k = extents[0]; k <= extents[1]; k++) // X
            {
                unsigned grid_index = this->mpRegularGrid->GetGlobalGridIndex(k, j, i);
                this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index,
                        p_linear_pde->ComputeDiscreteLinearInUCoeffInSourceTerm(grid_index)*reference_time);
            }
        }
    }
    this->mpLinearSystem->FinaliseLhsMatrix();
}

template<unsigned DIM>
void SimpleLinearEllipticFiniteDifferenceSolver<DIM>::AddDiscreteTermsToRhs()
{
    boost::shared_ptr<DiscreteContinuumLinearEllipticPde<DIM, DIM> > p_linear_pde =
                boost::dynamic_pointer_cast<DiscreteContinuumLinearEllipticPde<DIM, DIM> >(this->mpPde);
    if(!p_linear_pde)
    {
        EXCEPTION("PDE not recognized");
    }

    c_vector<unsigned, 6> extents = this->mpRegularGrid->GetExtents();
    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    for (unsigned i = extents[4]; i <= extents[5]; i++) // Z
    {
        for (unsigned j = extents[2]; j <= extents[3]; j++) // Y
        {
            for (unsigned k = extents[0]; k <= extents[1]; k++) // X
            {
                unsigned grid_index = this->mpRegularGrid->GetGlobalGridIndex(k, j, i);
                this->mpLinearSystem->SetRhsVectorElement(grid_index,
                        -p_linear_pde->ComputeDiscreteConstantInUSourceTerm(grid_index)*(reference_time/this->mReferenceConcentration));
            }
        }
    }
}

template<unsigned DIM>
void SimpleLinearEllipticFiniteDifferenceSolver<DIM>::AssembleMatrix()
{
    boost::shared_ptr<DiscreteContinuumLinearEllipticPde<DIM, DIM> > p_linear_pde =
                boost::dynamic_pointer_cast<DiscreteContinuumLinearEllipticPde<DIM, DIM> >(this->mpPde);

    if(!p_linear_pde)
    {
        EXCEPTION("PDE not recognized");
    }

    this->mpLinearSystem->ZeroLhsMatrix();

    c_vector<unsigned, 6> extents = this->mpRegularGrid->GetExtents();
    c_vector<unsigned, 3> dimensions = this->mpRegularGrid->GetDimensions();
    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();
    units::quantity<unit::length> spacing = this->mpRegularGrid->GetSpacing();
    double diffusion_term = (p_linear_pde->ComputeIsotropicDiffusionTerm() / (spacing * spacing))*reference_time;

    for (unsigned i = extents[4]; i <= extents[5]; i++) // Z
    {
        for (unsigned j = extents[2]; j <= extents[3]; j++) // Y
        {
            for (unsigned k = extents[0]; k <= extents[1]; k++) // X
            {
                unsigned grid_index = this->mpRegularGrid->GetGlobalGridIndex(k, j, i);
                this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index,
                        p_linear_pde->ComputeLinearInUCoeffInSourceTerm(grid_index)*reference_time - 6.0* diffusion_term);
                // No flux at x bottom
                if (k > 0)
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index - 1, diffusion_term);
                }
                else
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at x top
                if (k < dimensions[0] - 1)
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index + 1, diffusion_term);
                }
                else
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at y bottom
                if (j > 0)
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index - dimensions[0], diffusion_term);
                }
                else
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at y top
                if (j < dimensions[1] - 1)
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index + dimensions[0], diffusion_term);
                }
                else
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at z bottom
                if (i > 0)
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index - dimensions[0] * dimensions[1],
                            diffusion_term);
                }
                else
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }

                // No flux at z top
                if (i < dimensions[2] - 1)
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index + dimensions[0] * dimensions[1],
                            diffusion_term);
                }
                else
                {
                    this->mpLinearSystem->AddToMatrixElement(grid_index, grid_index, diffusion_term);
                }
            }
        }
    }
    this->mpLinearSystem->FinaliseLhsMatrix();
}

template<unsigned DIM>
void SimpleLinearEllipticFiniteDifferenceSolver<DIM>::AssembleVector()
{
    boost::shared_ptr<DiscreteContinuumLinearEllipticPde<DIM, DIM> > p_linear_pde =
                boost::dynamic_pointer_cast<DiscreteContinuumLinearEllipticPde<DIM, DIM> >(this->mpPde);
    if(!p_linear_pde)
    {
        EXCEPTION("PDE not recognized");
    }

    this->mpLinearSystem->ZeroRhsVector();
    c_vector<unsigned, 3> dimensions = this->mpRegularGrid->GetDimensions();
    c_vector<unsigned, 6> extents = this->mpRegularGrid->GetExtents();
    units::quantity<unit::time> reference_time = BaseUnits::Instance()->GetReferenceTimeScale();

    for (unsigned i = extents[4]; i <= extents[5]; i++) // Z
    {
        for (unsigned j = extents[2]; j <= extents[3]; j++) // Y
        {
            for (unsigned k = extents[0]; k <= extents[1]; k++) // X
            {
                unsigned grid_index = this->mpRegularGrid->GetGlobalGridIndex(k, j, i);
                this->mpLinearSystem->SetRhsVectorElement(grid_index,
                        -p_linear_pde->ComputeConstantInUSourceTerm(grid_index)*(reference_time/this->mReferenceConcentration));
            }
        }
    }
    this->mpLinearSystem->FinaliseRhsVector();
}

template<unsigned DIM>
void SimpleLinearEllipticFiniteDifferenceSolver<DIM>::Setup()
{
    AbstractFiniteDifferenceSolverBase<DIM>::Setup();

    // Set up the linear system
    Vec template_vec = this->mpRegularGrid->GetDistributedVectorFactory()->CreateVec();
    this->mpLinearSystem = boost::shared_ptr<LinearSystem>(new LinearSystem(template_vec, 7));
    PetscTools::Destroy(template_vec);

    // Assemble the system
    this->SetMatrixToAssemble(this->mpLinearSystem->rGetLhsMatrix());
    this->SetVectorToAssemble(this->mpLinearSystem->rGetRhsVector());
    this->AssembleMatrix();
    this->AssembleVector();

    // Store the system without discrete terms
    //MatCopy(this->mpLinearSystem->rGetLhsMatrix(), mpInitialLhs, DIFFERENT_NONZERO_PATTERN);
    //mpInitialLhs = this->mpLinearSystem->rGetLhsMatrix();
    //mpInitialRhs = this->mpLinearSystem->rGetRhsVector();
    // This will add the discrete terms and boundary conditions
    Update();
    this->IsSetupForSolve = true;
}

template<unsigned DIM>
void SimpleLinearEllipticFiniteDifferenceSolver<DIM>::Update()
{

    AbstractFiniteDifferenceSolverBase<DIM>::Update();

    // Copy over the original matrix
    this->mpLinearSystem->AssembleIntermediateLinearSystem();
    //MatCopy(mpInitialLhs, this->mpLinearSystem->rGetLhsMatrix(), DIFFERENT_NONZERO_PATTERN);
    //VecCopy(mpInitialRhs, this->mpLinearSystem->rGetRhsVector());
//
//    // Add discrete terms
    //this->AddDiscreteTermsToMatrix();
    //this->AddDiscreteTermsToRhs();

    // Add boundary conditions
    this->mpLinearSystem->AssembleIntermediateLinearSystem();
    std::vector<unsigned> bc_indices;
    unsigned lo = this->mpRegularGrid->GetDistributedVectorFactory()->GetLow();
    unsigned hi = this->mpRegularGrid->GetDistributedVectorFactory()->GetHigh();
    for(unsigned idx=lo; idx<hi; idx++)
    {
        if((*this->mpBoundaryConditions)[idx].first)
        {
            bc_indices.push_back(idx);
            this->mpLinearSystem->SetRhsVectorElement(idx, (*this->mpBoundaryConditions)[idx].second/this->mReferenceConcentration);
        }
    }
    this->mpLinearSystem->ZeroMatrixRowsWithValueOnDiagonal(bc_indices, 1.0);

    // Final Assembly
    this->mpLinearSystem->AssembleFinalLinearSystem();
}

template<unsigned DIM>
void SimpleLinearEllipticFiniteDifferenceSolver<DIM>::Solve()
{
    if(!this->IsSetupForSolve)
    {
        Setup();
    }

    // Do the solve
    ReplicatableVector soln_repl(this->mpLinearSystem->Solve());

    // Populate the solution vector
    c_vector<unsigned, 3> dimensions = this->mpRegularGrid->GetDimensions();
    unsigned number_of_points = dimensions[0]*dimensions[1]*dimensions[2];
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

// Explicit instantiation
template class SimpleLinearEllipticFiniteDifferenceSolver<2>;
template class SimpleLinearEllipticFiniteDifferenceSolver<3>;
