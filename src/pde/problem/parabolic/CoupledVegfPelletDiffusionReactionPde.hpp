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

#ifndef COUPLEDVEGFPELLETDIFFUSIONREACTIONPDE_HPP_
#define COUPLEDVEGFPELLETDIFFUSIONREACTIONPDE_HPP_

#include "ChastePoint.hpp"
#include "Element.hpp"
#include "SmartPointers.hpp"
#include "UnitCollection.hpp"
#include "AbstractDiscreteContinuumParabolicPde.hpp"

/**
 * Reaction diffusion PDE for VEGF leakage from a pellet. The pellet is treated as
 * a seperate non-spatial compartment.
 */
template<unsigned ELEMENT_DIM, unsigned SPACE_DIM = ELEMENT_DIM>
class CoupledVegfPelletDiffusionReactionPde : public AbstractDiscreteContinuumParabolicPde<ELEMENT_DIM, SPACE_DIM>
{
    /**
     * The linear in u source term
     */
    QRate mLinearInUTerm;

    /**
     * The non-linear source strengths for each point on the grid or mesh
     */
    std::vector<QConcentrationFlowRate > mDiscreteNonLinearSourceStrengths;

    /**
     * The decay rate of free VEGF in the pellet
     */
    QRate mPelletFreeDecayRate;

    /**
     * The binding constant for VEGF in the pellet
     */
    QDimensionless mPelletVegfBindingConstant;

    /**
     * The initial VEGF in the pellet
     */
    QConcentration mInitialVegfInPellet;

    /**
     * The current VEGF in the pellet
     */
    QConcentration mCurrentVegfInPellet;

    /**
     * The permeability of the cornea-pellet boundary
     */
    QMembranePermeability mCorneaPelletPermeability;

    /**
     * The pellet surface area
     */
    QArea mPelletSurfaceArea;

    /**
     * The pellet surface depth (for 2D problems)
     */
    QLength mPelletDepth;

    /**
     * The pellet volume
     */
    QVolume mPelletVolume;

    /**
     * Vegf concentration at half max receptor occupation
     */
    QConcentration mHalfMaxVegf;

public:

    /**
     * Constructor
     */
    CoupledVegfPelletDiffusionReactionPde();

    /**
     * Destructor
     */
    virtual ~CoupledVegfPelletDiffusionReactionPde();

    /**
     * Factory Constructor
     * @return a pointer to an instance of the pde
     */
    static std::shared_ptr<CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM> > Create();

    /**
     * @return computed source term.
     *
     * @param rX the point in space at which the nonlinear source term is computed
     * @param u the value of the dependent variable at the point
     * @param pElement the element that we are inside
     */
    virtual double ComputeSourceTerm(const ChastePoint<SPACE_DIM>& rX,
                                     double u,
                                     Element<ELEMENT_DIM,SPACE_DIM>* pElement=NULL);

    /**
     * Abstract method to return the non linear contribution to the regular grid solvers
     * @param gridIndex grid index
     * @param u the concentration
     * @return source strength
     */
    virtual QConcentrationFlowRate ComputeSourceTerm(unsigned gridIndex, QConcentration u);

    /**
     * Abstract method to return the non linear prime contribution to the regular grid solvers
     * @param gridIndex grid index
     * @param u the concentration
     * @return source strength
     */
    virtual QRate ComputeSourceTermPrime(unsigned gridIndex, QConcentration u);

    /**
     * Update the discrete source strengths
     */
    void UpdateDiscreteSourceStrengths();

    /**
     * Update the value of an optional coupling parameter
     */
    virtual void UpdateMultiplierValue();

    /**
     * @return the free vegf decay rate
     */
    QRate  GetPelletFreeDecayRate();

    /**
     * @return the pellet binding constant
     */
    QDimensionless  GetPelletBindingConstant();

    /**
     * @return the initial vegf in the pellet
     */
    QConcentration GetInitialVegfInPellet();

    /**
     * @return the current vegf in the pellet
     */
    QConcentration GetCurrentVegfInPellet();

    /**
     * @return the cornea pellet permeability
     */
    QMembranePermeability GetCorneaPelletPermeability();

    /**
     * @return the pellet surface area
     */
    QArea GetPelletSurfaceArea();

    /**
     * @return the pellet depth
     */
    QLength GetPelletDepth();

    /**
     * @return the pellet volume
     */
    QVolume GetPelletVolume();

    /**
     * Set the vegf decay rate
     * @param rate the vegf decay rate
     */
    void SetPelletFreeDecayRate(QRate rate);

    /**
     * Set the vegf binding constant
     * @param bindingConstant the vegf binding constant
     */
    void SetPelletBindingConstant(QDimensionless bindingConstant);

    /**
     * Set the initial vegf
     * @param initialVegf the initial vegf
     */
    void SetInitialVegfInPellet(QConcentration initialVegf);

    /**
     * Set the current vegf
     * @param currentVegf the current vegf
     */
    void SetCurrentVegfInPellet(QConcentration currentVegf);

    /**
     * Set cornea pellet permeability
     * @param permeability the cornea pellet permeability
     */
    void SetCorneaPelletPermeability(QMembranePermeability permeability);

    /**
     * Set the pellet surface area
     * @param surfaceArea the pellet surface area
     */
    void SetPelletSurfaceArea(QArea surfaceArea);

    /**
     * Set the pellet depth
     * @param depth the pellet depth
     */
    void SetPelletDepth(QLength depth);

    /**
     * Set the pellet volume
     * @param volume the pellet volume
     */
    void SetPelletVolume(QVolume volume);

    /**
     * Set the half max vegf concentration
     * @param halfMax the half max vegf concentration
     */
    void SetHalfMaxVegfConcentration(QConcentration halfMax);

    /**
     * Set the linear constant in U term
     * @param linearInUTerm the linear constant in U term
     */
    void SetContinuumLinearInUTerm(QRate linearInUTerm);

};

#endif /*COUPLEDVEGFPELLETDIFFUSIONREACTIONPDE_HPP_*/
