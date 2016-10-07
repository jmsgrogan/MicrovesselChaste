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

#include "AbstractMigrationRule.hpp"

template<unsigned DIM>
AbstractMigrationRule<DIM>::AbstractMigrationRule()
     :mpSolver(),
      mpVesselNetwork(),
      mIsSprouting(false),
      mpCellPopulation(),
      mpGrid(),
      mpBoundingDomain()
{

}

template<unsigned DIM>
AbstractMigrationRule<DIM>::~AbstractMigrationRule()
{

}

template <unsigned DIM>
boost::shared_ptr<AbstractMigrationRule<DIM> > AbstractMigrationRule<DIM>::Create()
{
    MAKE_PTR(AbstractMigrationRule<DIM>, pSelf);
    return pSelf;
}

template <unsigned DIM>
std::vector<DimensionalChastePoint<DIM> > AbstractMigrationRule<DIM>::GetDirections(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    return std::vector<DimensionalChastePoint<DIM> >();
}

template <unsigned DIM>
std::vector<int> AbstractMigrationRule<DIM>::GetIndices(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes)
{
    return std::vector<int>();
}

template<unsigned DIM>
void AbstractMigrationRule<DIM>::SetBoundingDomain(boost::shared_ptr<Part<DIM> > pPart)
{
    mpBoundingDomain = pPart;
}

template<unsigned DIM>
void AbstractMigrationRule<DIM>::SetIsSprouting(bool isSprouting)
{
    mIsSprouting = isSprouting;
}

template<unsigned DIM>
void AbstractMigrationRule<DIM>::SetGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid)
{
    mpGrid = pGrid;
}

template<unsigned DIM>
void AbstractMigrationRule<DIM>::SetDiscreteContinuumSolver(boost::shared_ptr<AbstractDiscreteContinuumSolver<DIM> > pSolver)
{
    mpSolver = pSolver;
}

template<unsigned DIM>
void AbstractMigrationRule<DIM>::SetNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpVesselNetwork = pNetwork;
}

template<unsigned DIM>
void AbstractMigrationRule<DIM>::SetCellPopulation(boost::shared_ptr<AbstractCellPopulation<DIM> > pPopulation)
{
    mpCellPopulation = pPopulation;
}

// Explicit instantiation
template class AbstractMigrationRule<2> ;
template class AbstractMigrationRule<3> ;
