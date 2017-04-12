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

#include <boost/lexical_cast.hpp>
#include "AbstractDiscreteContinuumParabolicPde.hpp"
#include "CoupledLumpedSystemFiniteElementSolver.hpp"
#include "CoupledInterfaceOdePdeSolver.hpp"
#include "Exception.hpp"
#include "SimpleLinearParabolicSolverWithStorage.hpp"

template<unsigned DIM>
CoupledLumpedSystemFiniteElementSolver<DIM>::CoupledLumpedSystemFiniteElementSolver()
    : AbstractFiniteElementSolverBase<DIM>(),
      mIntermediateSolutionCollection(),
      mIntermediateSolutionFrequency(1),
      mStoreIntermediate(false),
      mWriteIntermediate(false),
      mTimeIncrement(0.1),
      mSolveStartTime(0.0),
      mSolveEndTime(1.0),
      mInitialGuess(),
      mUseCoupling(true)
{

}

template<unsigned DIM>
CoupledLumpedSystemFiniteElementSolver<DIM>::~CoupledLumpedSystemFiniteElementSolver()
{

}

template <unsigned DIM>
boost::shared_ptr<CoupledLumpedSystemFiniteElementSolver<DIM> > CoupledLumpedSystemFiniteElementSolver<DIM>::Create()
{
    MAKE_PTR(CoupledLumpedSystemFiniteElementSolver<DIM>, pSelf);
    return pSelf;
}

template <unsigned DIM>
void CoupledLumpedSystemFiniteElementSolver<DIM>::SetTargetTimeIncrement(double targetIncrement)
{
    mTimeIncrement = targetIncrement;
}

template <unsigned DIM>
void CoupledLumpedSystemFiniteElementSolver<DIM>::SetStartTime(double startTime)
{
    mSolveStartTime = startTime;
}

template <unsigned DIM>
void CoupledLumpedSystemFiniteElementSolver<DIM>::SetEndTime(double endTime)
{
    mSolveEndTime = endTime;
}

template <unsigned DIM>
void CoupledLumpedSystemFiniteElementSolver<DIM>::SetInitialGuess(const std::vector<double>& rInitialGuess)
{
    mInitialGuess = rInitialGuess;
}

template<unsigned DIM>
void CoupledLumpedSystemFiniteElementSolver<DIM>::SetUseCoupling(bool useCoupling)
{
    mUseCoupling = useCoupling;
}

template <unsigned DIM>
const std::vector<std::pair<std::vector<double>, double> >& CoupledLumpedSystemFiniteElementSolver<DIM>::rGetIntermediateSolutions()
{
    return mIntermediateSolutionCollection;
}

template <unsigned DIM>
void CoupledLumpedSystemFiniteElementSolver<DIM>::SetStoreIntermediateSolutions(bool store, unsigned frequency)
{
    mStoreIntermediate = store;
    mIntermediateSolutionFrequency = frequency;
}

template <unsigned DIM>
void CoupledLumpedSystemFiniteElementSolver<DIM>::SetWriteIntermediateSolutions(bool write, unsigned frequency)
{
    mWriteIntermediate = write;
    mStoreIntermediate = write;
    mIntermediateSolutionFrequency = frequency;
}

template<unsigned DIM>
void CoupledLumpedSystemFiniteElementSolver<DIM>::Solve()
{
    AbstractFiniteElementSolverBase<DIM>::Solve();

    // Set up the boundary conditions in the Chaste format
    boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, 1> > p_bcc =
            boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, 1> >(new BoundaryConditionsContainer<DIM, DIM, 1> );

    for(unsigned idx=0; idx<this->mBoundaryConditions.size(); idx++)
    {
        this->mBoundaryConditions[idx]->SetGridCalculator(this->mpDensityMap->GetGridCalculator());
        this->mBoundaryConditions[idx]->UpdateBoundaryConditions(p_bcc);
    }

    // Check the type of pde
    if(boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<DIM, DIM> > p_parabolic_pde =
            boost::dynamic_pointer_cast<CoupledVegfPelletDiffusionReactionPde<DIM, DIM> >(this->mpPde))
    {
        Vec initial_guess = PetscTools::CreateAndSetVec(this->mpMesh->GetNumNodes(), 0.0);
        CoupledInterfaceOdePdeSolver<DIM> solver(this->mpMesh.get(), p_bcc.get(), p_parabolic_pde.get());

        /* The interface is exactly the same as the `SimpleLinearParabolicSolver`. */
        solver.SetTimeStep(mTimeIncrement);
        solver.SetTimes(mSolveStartTime, mSolveEndTime);
        solver.SetInitialCondition(initial_guess);
        solver.SetStoreIntermediateSolutions(mStoreIntermediate, mIntermediateSolutionFrequency);
        solver.SetUseCoupling(mUseCoupling);
        solver.SetInitialDimensionlessLumpedSolution(p_parabolic_pde->GetCurrentVegfInPellet()/this->mReferenceConcentration);

        // Set the dimensionless permeability - remember the robin boundary condition does not contain the diffusion term.
        double permeability = p_parabolic_pde->GetCorneaPelletPermeability()*(BaseUnits::Instance()->GetReferenceTimeScale()/this->mpMesh->GetReferenceLengthScale());
        solver.SetDimensionlessPermeability(permeability);
        solver.SetReferenceLengthScale(this->mpMesh->GetReferenceLengthScale());

        Vec result = solver.Solve();
        ReplicatableVector solution_repl(result);

        this->mSolution = std::vector<double>(solution_repl.GetSize());
        this->mConcentrations = std::vector<units::quantity<unit::concentration> >(solution_repl.GetSize());
        for(unsigned idx = 0; idx < solution_repl.GetSize(); idx++)
        {
            this->mSolution[idx] = solution_repl[idx];
            this->mConcentrations[idx] = solution_repl[idx]*this->mReferenceConcentration;
        }
        this->UpdateSolution(this->mSolution);

        if(mStoreIntermediate)
        {
            mIntermediateSolutionCollection = solver.rGetIntermediateSolutions();
        }

        // Tidy up
        PetscTools::Destroy(initial_guess);
        PetscTools::Destroy(result);
    }
    else
    {
        EXCEPTION("PDE Type could not be identified, did you set a PDE?");
    }

    if(this->mWriteSolution)
    {
        this->Write();
    }

    if(mWriteIntermediate)
    {
        std::string base_file_name;
        std::string original_file_name;
        if(this->mFilename.empty())
        {
            original_file_name = "solution";
        }
        else
        {
            original_file_name = this->mFilename;
        }
        base_file_name = original_file_name + "_intermediate_t_";

        for(unsigned idx=0;idx<mIntermediateSolutionCollection.size();idx++)
        {
            this->UpdateSolution(mIntermediateSolutionCollection[idx].first);
            this->mFilename = base_file_name + boost::lexical_cast<std::string>(unsigned(100.0*mIntermediateSolutionCollection[idx].second/mTimeIncrement));
            this->Write();
        }
        this->mFilename = original_file_name;
    }
}

// Explicit instantiation
template class CoupledLumpedSystemFiniteElementSolver<2>;
template class CoupledLumpedSystemFiniteElementSolver<3>;
