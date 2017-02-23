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
 * Redistributions in binary form must reproduce the abovea copyright notice,
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
     * The non-linear source strengths for each point on the grid or mesh
     */
    std::vector<units::quantity<unit::concentration_flow_rate> > mDiscreteNonLinearSourceStrengths;

    /**
     * The decay rate of free VEGF in the pellet
     */
    units::quantity<unit::rate> mPelletFreeDecayRate;

    /**
     * The binding constant for VEGF in the pellet
     */
    units::quantity<unit::dimensionless> mPelletVegfBindingConstant;

    /**
     * The initial VEGF in the pellet
     */
    units::quantity<unit::concentration> mInitialVegfInPellet;

    /**
     * The permeability of the cornea-pellet boundary
     */
    units::quantity<unit::membrane_permeability> mCorneaPelletPermeability;

    /**
     * The pellet surface area
     */
    units::quantity<unit::area> mPelletSurfaceArea;

    /**
     * The pellet volume
     */
    units::quantity<unit::volume> mPelletVolume;

    /**
     * Vegf concentration at half max receptor occupation
     */
    units::quantity<unit::concentration> mHalfMaxVegf;

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
    static boost::shared_ptr<CoupledVegfPelletDiffusionReactionPde<ELEMENT_DIM, SPACE_DIM> > Create();

    /**
     * Overwritten method to return the linear in U contribution to the Chaste FE solver
     * @param rX grid location
     * @param pElement pointer to containing element
     * @return source strength
     */
    double ComputeLinearInUCoeffInSourceTerm(const ChastePoint<SPACE_DIM>& rX, Element<ELEMENT_DIM, SPACE_DIM>* pElement);

    /**
     * Overwritten method to return the linear in U contribution to the regular grid solvers
     * @param gridIndex grid index
     * @return source strength
     */
    units::quantity<unit::rate> ComputeLinearInUCoeffInSourceTerm(unsigned gridIndex=0);

    /**
     * Abstract method to return the non linear contribution to the regular grid solvers
     * @param gridIndex grid index
     * @param u the concentration
     * @return source strength
     */
    virtual units::quantity<unit::concentration_flow_rate> ComputeNonlinearSourceTerm(unsigned gridIndex, units::quantity<unit::concentration> u);

    /**
     * Abstract method to return the non linear prime contribution to the regular grid solvers
     * @param gridIndex grid index
     * @param u the concentration
     * @return source strength
     */
    virtual units::quantity<unit::rate> ComputeNonlinearSourceTermPrime(unsigned gridIndex, units::quantity<unit::concentration> u);

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
    units::quantity<unit::rate>  GetPelletFreeDecayRate();

    /**
     * @return the pellet binding constant
     */
    units::quantity<unit::dimensionless>  GetPelletBindingConstant();

    /**
     * @return the initial vegf in the pellet
     */
    units::quantity<unit::concentration> GetInitialVegfInPellet();

    /**
     * @return the cornea pellet permeability
     */
    units::quantity<unit::membrane_permeability> GetCorneaPelletPermeability();

    /**
     * @return the pellet surface area
     */
    units::quantity<unit::area> GetPelletSurfaceArea();

    /**
     * @return the pellet volume
     */
    units::quantity<unit::volume> GetPelletVolume();

    /**
     * Set the vegf decay rate
     * @param rate the vegf decay rate
     */
    void SetPelletFreeDecayRate(units::quantity<unit::rate> rate);

    /**
     * Set the vegf binding constant
     * @param bindingConstant the vegf binding constant
     */
    void SetPelletBindingConstant(units::quantity<unit::dimensionless> bindingConstant);

    /**
     * Set the initial vegf
     * @param initialVegf the initial vegf
     */
    void SetInitialVegfInPellet(units::quantity<unit::concentration> initialVegf);

    /**
     * Set cornea pellet permeability
     * @param permeability the cornea pellet permeability
     */
    void SetCorneaPelletPermeability(units::quantity<unit::membrane_permeability> permeability);

    /**
     * Set the pellet surface area
     * @param surfaceArea the pellet surface area
     */
    void SetPelletSurfaceArea(units::quantity<unit::area> surfaceArea);

    /**
     * Set the pellet volume
     * @param volume the pellet volume
     */
    void SetPelletVolume(units::quantity<unit::volume> volume);

    /**
     * Set the half max vegf concentration
     * @param halfMax the half max vegf concentration
     */
    void SetHalfMaxVegfConcentration(units::quantity<unit::concentration> halfMax);

};

#endif /*COUPLEDVEGFPELLETDIFFUSIONREACTIONPDE_HPP_*/
