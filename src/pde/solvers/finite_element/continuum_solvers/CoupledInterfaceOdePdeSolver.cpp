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

#include "CoupledInterfaceOdePdeSolver.hpp"
#include "CoupledVegfPelletDiffusionReactionPde.hpp"

/**
 * A Parabolic PDE with a coupled ODE system on prescribed boundary nodes
 */
template<unsigned DIM>
CoupledInterfaceOdePdeSolver<DIM>::CoupledInterfaceOdePdeSolver(TetrahedralMesh<DIM,DIM>* pMesh,
                           BoundaryConditionsContainer<DIM,DIM,1>* pBoundaryConditions,
                           AbstractLinearParabolicPde<DIM,DIM>* pPde)
     : AbstractDynamicLinearPdeSolver<DIM,DIM,1>(pMesh),
       mpParabolicPde(pPde),
       mIntermediateSolutionCollection(),
       mIntermediateSolutionFrequency(1),
       mStoreIntermediate(false),
       mIncrementCounter(0),
       mCurrentTime(0.0),
       mpBoundaryConditions(pBoundaryConditions),
       mPermeability(1.0),
       mCurrentLumpedSolution(0.0),
       mReferenceLengthScale(BaseUnits::Instance()->GetReferenceLengthScale()),
       mUseCoupling(true)
{
    this->mMatrixIsConstant = true;
}

template<unsigned DIM>
CoupledInterfaceOdePdeSolver<DIM>::~CoupledInterfaceOdePdeSolver()
{

}

template<unsigned DIM>
void CoupledInterfaceOdePdeSolver<DIM>::SetStoreIntermediateSolutions(bool store,
        unsigned frequency)
{
    mStoreIntermediate = store;
    mIntermediateSolutionFrequency = frequency;
}

template<unsigned DIM>
void CoupledInterfaceOdePdeSolver<DIM>::SetUseCoupling(bool useCoupling)
{
    mUseCoupling = useCoupling;
}

template<unsigned DIM>
const std::vector<std::pair<std::vector<double>, double> >& CoupledInterfaceOdePdeSolver<DIM>::rGetIntermediateSolutions()
{
    return mIntermediateSolutionCollection;
}

template<unsigned DIM>
void CoupledInterfaceOdePdeSolver<DIM>::SetInitialDimensionlessLumpedSolution(double solution)
{
    mCurrentLumpedSolution = solution;
}

template<unsigned DIM>
void CoupledInterfaceOdePdeSolver<DIM>::SetDimensionlessPermeability(double permeability)
{
    mPermeability=permeability;
}

template<unsigned DIM>
void CoupledInterfaceOdePdeSolver<DIM>::SetReferenceLengthScale(QLength referenceLengthScale)
{
    mReferenceLengthScale = referenceLengthScale;
}

template<unsigned DIM>
void CoupledInterfaceOdePdeSolver<DIM>::SetupLinearSystem(Vec currentSolution, bool computeMatrix)
{
    // Assemble the parabolic terms
    CoupledOdePdeParabolicTermAssembler<DIM> parabolic_terms_assembler(this->mpMesh, this->mpParabolicPde);
    CoupledVegfPelletDiffusionReactionPde<DIM, DIM>* p_coupled_pde = dynamic_cast<CoupledVegfPelletDiffusionReactionPde<DIM, DIM>*>(this->mpParabolicPde);

    // Set the robin
    if (computeMatrix)
    {
        parabolic_terms_assembler.SetMatrixToAssemble(this->mpLinearSystem->rGetLhsMatrix());
        parabolic_terms_assembler.AssembleMatrix();

        this->mpLinearSystem->FinaliseLhsMatrix(); // (Petsc communication)

        RobinConditionsSurfaceTermAssembler<DIM,DIM,1> surface_integral_assembler(this->mpMesh, mpBoundaryConditions);
        surface_integral_assembler.SetMatrixToAssemble(this->mpLinearSystem->rGetLhsMatrix(), false);
        surface_integral_assembler.SetPermeability(mPermeability);
        surface_integral_assembler.AssembleMatrix();

        this->mpLinearSystem->FinaliseLhsMatrix(); // (Petsc communication)
    }
    else
    {
        // Update the robin boundary condition value
        RobinConditionsSurfaceTermAssembler<DIM,DIM,1> surface_integral_assembler(this->mpMesh, mpBoundaryConditions);
        surface_integral_assembler.SetVectorToAssemble(this->mpLinearSystem->rGetRhsVector(), true);
        surface_integral_assembler.SetPermeability(mPermeability);
        units::quantity<unit::dimensionless> binding_constant = p_coupled_pde->GetPelletBindingConstant();
        surface_integral_assembler.SetMultiplier(double(mCurrentLumpedSolution/binding_constant));
        surface_integral_assembler.AssembleVector();
        this->mpLinearSystem->FinaliseRhsVector(); // (Petsc communication)

        parabolic_terms_assembler.SetVectorToAssemble(this->mpLinearSystem->rGetRhsVector(), false);
        parabolic_terms_assembler.SetCurrentSolution(currentSolution);
        parabolic_terms_assembler.AssembleVector();
        this->mpLinearSystem->FinaliseRhsVector(); // (Petsc communication)

        if(mUseCoupling)
        {
            // Update the VEGF in the pellet to the end of the timestep
            NullSurfaceIntegralCalculator<DIM> surf_calc(this->mpMesh, mpBoundaryConditions);
            surf_calc.SetCurrentSolution(currentSolution);

            QLength depth=mReferenceLengthScale;
            if(DIM==2)
            {
                depth=p_coupled_pde->GetPelletDepth();
            }
            units::quantity<unit::area> surface_area = surf_calc.CalculateSurfaceIntegral(false)*mReferenceLengthScale*depth;
            units::quantity<unit::area> surface_concentration_integral = surf_calc.CalculateSurfaceIntegral()*mReferenceLengthScale*depth;

            // Update the vegf in the pellet
            units::quantity<unit::volume> volume = p_coupled_pde->GetPelletVolume();
            units::quantity<unit::membrane_permeability> permeability = p_coupled_pde->GetCorneaPelletPermeability();
            units::quantity<unit::dimensionless> binding_constant = p_coupled_pde->GetPelletBindingConstant();
            units::quantity<unit::rate> decay_rate = p_coupled_pde->GetPelletFreeDecayRate();
            units::quantity<unit::rate> pellet_update_term1 = (permeability/volume)*((surface_area*mCurrentLumpedSolution/binding_constant) -surface_concentration_integral);
            units::quantity<unit::rate> dVegf_dt = -(decay_rate/binding_constant)*mCurrentLumpedSolution - pellet_update_term1;
            mCurrentLumpedSolution += dVegf_dt*PdeSimulationTime::GetPdeTimeStep()*BaseUnits::Instance()->GetReferenceTimeScale();
        }
    }

    /* Some necessary PETSc communication before applying Dirichet BCs */
    this->mpLinearSystem->FinaliseRhsVector();         // (Petsc communication)
    this->mpLinearSystem->SwitchWriteModeLhsMatrix();  // (Petsc communication - needs to called when going from adding entries to inserting entries)

    /* Apply the dirichlet BCs from the BCC to the linear system */
    mpBoundaryConditions->ApplyDirichletToLinearProblem(*(this->mpLinearSystem), computeMatrix);

    /* Some necessary PETSc communication to finish */
    this->mpLinearSystem->FinaliseRhsVector();
    this->mpLinearSystem->FinaliseLhsMatrix();
}

template<unsigned DIM>
double CoupledInterfaceOdePdeSolver<DIM>::GetDimensionlessLumpedSolution()
{
   return mCurrentLumpedSolution;
}

template<unsigned DIM>
void CoupledInterfaceOdePdeSolver<DIM>::FollowingSolveLinearSystem(Vec currentSolution)
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
template class CoupledInterfaceOdePdeSolver<2>;
template class CoupledInterfaceOdePdeSolver<3>;
