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

    AbstractLinearParabolicPde<DIM,DIM>* mpParabolicPde;

private:

    /* The constuctor will take in a mesh and a BCC, the latter will be stored as a member variable */
    BoundaryConditionsContainer<DIM,DIM,1>* mpBoundaryConditions;

    Mat mRhsRobinMatrix;

    double mPermeability;

    double mCurrentVegfSolution;

    /* This is the main method which needs to be implemented. It takes in the current solution, and a
     * boolean saying whether the matrix (ie A in Ax=b) is being computed or not.
     */
    void SetupLinearSystem(Vec currentSolution, bool computeMatrix)
    {
        // Update the surface integral value
        NullSurfaceIntegralCalculator<DIM> surf_calc(this->mpMesh, mpBoundaryConditions);
        surf_calc.SetCurrentSolution(currentSolution);
        double integral_value = surf_calc.CalculateSurfaceIntegral();

//        // Update the vegf in the pellet
//        boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<DIM, DIM> > p_coupled_pde =
//                    boost::dynamic_pointer_cast<CoupledVegfPelletDiffusionReactionPde<DIM, DIM> >(this->mpParabolicPde);
//
//        units::quantity<unit::volume> volume = p_coupled_pde->GetPelletVolume();
//        units::quantity<unit::membrane_permeability> permeability = p_coupled_pde->GetCorneaPelletPermeability();
//        units::quantity<unit::dimensionless> binding_constant = p_coupled_pde->GetPelletBindingConstant();
//        units::quantity<unit::rate> decay_rate = p_coupled_pde->GetPelletFreeDecayRate();
//        double pellet_solution = soln_guess_repl[num_points];
//        units::quantity<unit::rate> pellet_update_term1 = ((surface_area*permeability)/volume)*((mCurrentVegfSolution/binding_constant) -
//                integral_value);
//        units::quantity<unit::rate> dVegf_dt = -(decay_rate/binding_constant)*mCurrentVegfSolution - pellet_update_term1;
//        mCurrentVegfSolution += dVegf_dt*time;


        // Assemble the parabolic terms
        CoupledOdePdeParabolicTermAssembler<DIM> parabolic_terms_assembler(this->mpMesh, this->mpParabolicPde);

        if (computeMatrix)
        {
            parabolic_terms_assembler.SetMatrixToAssemble(this->mpLinearSystem->rGetLhsMatrix());
            parabolic_terms_assembler.AssembleMatrix();
            this->mpLinearSystem->FinaliseLhsMatrix(); // (Petsc communication)
        }
        else
        {
            RobinConditionsSurfaceTermAssembler<DIM,DIM,1> surface_integral_assembler(this->mpMesh, mpBoundaryConditions);
            surface_integral_assembler.SetMatrixToAssemble(mRhsRobinMatrix, true);
            surface_integral_assembler.AssembleMatrix();
            PetscMatTools::Finalise(mRhsRobinMatrix);
            MatMult(mRhsRobinMatrix, currentSolution, this->mpLinearSystem->rGetRhsVector());

            surface_integral_assembler.SetVectorToAssemble(this->mpLinearSystem->rGetRhsVector(), false);
            surface_integral_assembler.AssembleVector();
            this->mpLinearSystem->FinaliseRhsVector(); // (Petsc communication)

            parabolic_terms_assembler.SetVectorToAssemble(this->mpLinearSystem->rGetRhsVector(), false);
            parabolic_terms_assembler.SetCurrentSolution(currentSolution);
            parabolic_terms_assembler.AssembleVector();
            this->mpLinearSystem->FinaliseRhsVector(); // (Petsc communication)
        }

//        /* The third assembler we use is the `NaturalNeumannSurfaceTermAssembler`, which assembles
//         * the vector `c` defined above, using the Neumann BCs stored in the `BoundaryConditionsContainer`
//         * which is passed in in the constructor
//         */
//        NaturalNeumannSurfaceTermAssembler<DIM,DIM,1> surface_integral_assembler(this->mpMesh, mpBoundaryConditions);
//        surface_integral_assembler.SetVectorToAssemble(this->mpLinearSystem->rGetRhsVector(), false /*don't zero vector before assembling!*/);
//        surface_integral_assembler.Assemble();

        /* Some necessary PETSc communication before applying Dirichet BCs */
        this->mpLinearSystem->FinaliseRhsVector();         // (Petsc communication)
        this->mpLinearSystem->SwitchWriteModeLhsMatrix();  // (Petsc communication - needs to called when going from adding entries to inserting entries)

        /* Apply the dirichlet BCs from the BCC to the linear system */
        mpBoundaryConditions->ApplyDirichletToLinearProblem(*(this->mpLinearSystem), computeMatrix);

        /* Some necessary PETSc communication to finish */
        this->mpLinearSystem->FinaliseRhsVector();
        this->mpLinearSystem->FinaliseLhsMatrix();

        // Update the Pellet VEGF for the next step
    }

public:
    /* The constructor needs to call the parent constructor, save the BCC, ''say that the (LHS) matrix is constant
     * in time'' (so it is only computed once), and allocate memory for the RHS matrix.
     */
    CoupledInterfaceOdePdeSolver(TetrahedralMesh<DIM,DIM>* pMesh,
                               BoundaryConditionsContainer<DIM,DIM,1>* pBoundaryConditions,
                               AbstractLinearParabolicPde<DIM,DIM>* pPde)
         : AbstractDynamicLinearPdeSolver<DIM,DIM,1>(pMesh),
           mpParabolicPde(pPde),
           mpBoundaryConditions(pBoundaryConditions),
           mPermeability(1.0),
           mCurrentVegfSolution(0.0)
    {
        this->mMatrixIsConstant = true;
        PetscTools::SetupMat(mRhsRobinMatrix, this->mpMesh->GetNumNodes(), this->mpMesh->GetNumNodes(), 9);
    }

    ~CoupledInterfaceOdePdeSolver()
    {
        PetscTools::Destroy(mRhsRobinMatrix);
    }

    /**
     * Over-ridden method to
     *
     * @param currentSolution The current solution (solution of the linear system solve)
     */
    virtual void FollowingSolveLinearSystem(Vec currentSolution)
    {
    }

};

#endif /* COUPLEDINTERFACEODEPDESOLVER */
