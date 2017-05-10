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

#ifndef _BetteridgeHaematocritSolver_hpp
#define _BetteridgeHaematocritSolver_hpp

#include "SmartPointers.hpp"
#include "AbstractHaematocritSolver.hpp"
#include "UnitCollection.hpp"

/**
 * This solver calculates the distribution of haematocrit in branching vessel networks according to:
 * Betteridge et al. (2006), Networks and Heterogenous Media, 1(4), pp515-535.
 */
template<unsigned DIM>
class BetteridgeHaematocritSolver : public AbstractHaematocritSolver<DIM>
{

private:

    /**
     * The threshold velocity ratio at which all haematocrit goes into the faster vessel
     */
    units::quantity<unit::dimensionless> mTHR;

    /**
     * The partition coefficient for haematocrit
     */
    units::quantity<unit::dimensionless> mAlpha;

    /**
     * The arterial haematocrit level
     */
    units::quantity<unit::dimensionless> mHaematocrit;

    /**
     * Attempt to solve networks with connectivity > 3. Not in original model.
     * Haematocrit splits according to flow rate ratio only in higher connectivity cases.
     */
    bool mSolveHighConnectivityNetworks;

    /**
     * Turn off Fung Model. Haematocrit splits according to flow rate ratio only.
     */
    bool mTurnOffFungModel;

    /**
     * Use a random splitting model
     */
    bool mUseRandomSplitting;

    /**
     * Throw an exception if convergence fails
     */
    bool mExceptionOnFailedConverge;

public:

    /**
     * Constructor.
     */
    BetteridgeHaematocritSolver();

    /**
     *  destructor.
     */
    ~BetteridgeHaematocritSolver();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new class instance
     */
    static boost::shared_ptr<BetteridgeHaematocritSolver<DIM> > Create();

    /**
     *  Do the solve
     */
    void Calculate();

    void SetExceptionOnFailedConverge(bool setException);

    /**
     * Set the threshold velocity ratio
     * @param thr the threshold velocity ratio
     */
    void SetTHR(units::quantity<unit::dimensionless> thr);

    /**
     * Set the partition coefficient for haematocrit
     * @param alpha the partition coefficient for haematocrit
     */
    void SetAlpha(units::quantity<unit::dimensionless> alpha);

    /**
     * Set the artial haematocrit
     * @param haematocrit the arterial haematocrit
     */
    void SetHaematocrit(units::quantity<unit::dimensionless> haematocrit);

    /**
     * Attempt to use higher connectivity branches
     * @param useHighConnectivity use higher connectivity branches
     */
    void SetUseHigherConnectivityBranches(bool useHighConnectivity);

    /**
     * Turn off Fung Model. Haematocrit splits according to flow rate ratio only.
     * @param turnOffFungModel Haematocrit splits according to flow rate ratio only.
     */
    void SetTurnOffFungModel(bool turnOffFungModel);

    /**
     * Turn off Fung Model. Haematocrit splits according to flow rate ratio only.
     * @param turnOffFungModel Haematocrit splits according to flow rate ratio only.
     */
    void SetUseRandomSplittingModel(bool useRandomSplittingModel);

};

#endif
