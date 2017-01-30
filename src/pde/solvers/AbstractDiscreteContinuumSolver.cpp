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

#include "AbstractDiscreteContinuumSolver.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
AbstractDiscreteContinuumSolver<DIM>::AbstractDiscreteContinuumSolver()
    :   mpNetwork(),
        mpCellPopulation(NULL),
        mCellPopulationReferenceLength(BaseUnits::Instance()->GetReferenceLengthScale()),
        mCellPopulationReferenceConcentration(BaseUnits::Instance()->GetReferenceConcentrationScale()),
        mpOutputFileHandler(),
        mFilename(),
        mLabel("Default"),
        IsSetupForSolve(false),
        mWriteSolution(false),
        mpPde(),
        mpNonLinearPde(),
        mpParabolicPde(),
        mBoundaryConditions(),
        mReferenceConcentration(BaseUnits::Instance()->GetReferenceConcentrationScale()),
        mSolution(),
        mConcentrations(),
        mHasRegularGrid(false),
        mHasUnstructuredGrid(false)
{

}

template<unsigned DIM>
AbstractDiscreteContinuumSolver<DIM>::~AbstractDiscreteContinuumSolver()
{

}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::AddBoundaryCondition(boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > pBoundaryCondition)
{
    mBoundaryConditions.push_back(pBoundaryCondition);
}

template<unsigned DIM>
bool AbstractDiscreteContinuumSolver<DIM>::CellPopulationIsSet()
{
    return bool(mpCellPopulation);
}

template<unsigned DIM>
std::vector<units::quantity<unit::concentration> > AbstractDiscreteContinuumSolver<DIM>::GetConcentrations()
{
    return mConcentrations;
}

template<unsigned DIM>
const std::string& AbstractDiscreteContinuumSolver<DIM>::GetLabel()
{
    return mLabel;
}

template<unsigned DIM>
boost::shared_ptr<AbstractDiscreteContinuumLinearEllipticPde<DIM, DIM> > AbstractDiscreteContinuumSolver<DIM>::GetPde()
{
    if(!mpPde)
    {
        EXCEPTION("A pde has not been set.");
    }
    return mpPde;
}

template<unsigned DIM>
boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > AbstractDiscreteContinuumSolver<DIM>::GetNonLinearPde()
{
    if(!mpNonLinearPde)
    {
        EXCEPTION("A nonlinear pde has not been set.");
    }
    return mpNonLinearPde;
}

template<unsigned DIM>
boost::shared_ptr<AbstractDiscreteContinuumParabolicPde<DIM, DIM> > AbstractDiscreteContinuumSolver<DIM>::GetParabolicPde()
{
    if(!mpParabolicPde)
    {
        EXCEPTION("A parabolic pde has not been set.");
    }
    return mpParabolicPde;
}

template<unsigned DIM>
units::quantity<unit::concentration> AbstractDiscreteContinuumSolver<DIM>::GetReferenceConcentration()
{
    return mReferenceConcentration;
}

template<unsigned DIM>
std::vector<double> AbstractDiscreteContinuumSolver<DIM>::GetSolution()
{
    return mSolution;
}

template<unsigned DIM>
bool AbstractDiscreteContinuumSolver<DIM>::HasRegularGrid()
{
    return mHasRegularGrid;
}

template<unsigned DIM>
bool AbstractDiscreteContinuumSolver<DIM>::HasUnstructuredGrid()
{
    return mHasUnstructuredGrid;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation,
                                                             units::quantity<unit::length> cellPopulationReferenceLength,
                                                             units::quantity<unit::concentration> cellPopulationReferenceConcentration)
{
    mpCellPopulation = &rCellPopulation;
    mCellPopulationReferenceLength = cellPopulationReferenceLength;
    mCellPopulationReferenceConcentration = cellPopulationReferenceConcentration;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetFileHandler(boost::shared_ptr<OutputFileHandler> pOutputFileHandler)
{
    mpOutputFileHandler = pOutputFileHandler;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetFileName(const std::string& rFilename)
{
    mFilename = rFilename;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetLabel(const std::string& rLabel)
{
    mLabel = rLabel;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetPde(boost::shared_ptr<AbstractDiscreteContinuumLinearEllipticPde<DIM, DIM> > pPde)
{
    mpPde = pPde;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetNonLinearPde(boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > pPde)
{
    mpNonLinearPde = pPde;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetParabolicPde(boost::shared_ptr<AbstractDiscreteContinuumParabolicPde<DIM, DIM> > pPde)
{
    mpParabolicPde = pPde;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetReferenceConcentration(units::quantity<unit::concentration> referenceConcentration)
{
    mReferenceConcentration = referenceConcentration;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpNetwork = pNetwork;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::SetWriteSolution(bool write)
{
    mWriteSolution = write;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::UpdateSolution(const std::vector<double>& data)
{
    mSolution = data;
}

template<unsigned DIM>
void AbstractDiscreteContinuumSolver<DIM>::UpdateSolution(const std::vector<units::quantity<unit::concentration> >& data)
{
    mConcentrations = data;
}

// Explicit instantiation
template class AbstractDiscreteContinuumSolver<2> ;
template class AbstractDiscreteContinuumSolver<3> ;
