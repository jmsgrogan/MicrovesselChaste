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

#ifndef _MECHANICALSTIMULUSCALCULATOR_HPP
#define _MECHANICALSTIMULUSCALCULATOR_HPP

#include "SmartPointers.hpp"
#include "AbstractVesselNetworkCalculator.hpp"
#include "UnitCollection.hpp"

/**
 * This solver calculates a flow derived vessel growth stimulus according to:
 * Alarcon et al. (2003), JTB, 225, pp257-274. This Calculator has been changed from the original found in Pries1998
 * in order to better fit experimental data. See original paper and relevant test for comparison.
 *
 * The calculator assumes pressures are in Pa
 */
template<unsigned DIM>
class MechanicalStimulusCalculator : public AbstractVesselNetworkCalculator<DIM>
{
    
private:
    
	/**
	 * A small constant included to avoid singular behavior at low wall shear stress.
	 */
    QPressure mTauRef;

    /**
     * The level of wall shear stress expected from the actual intravascular pressure, according
     * to a parametric description of experimental data obtained in the rat mesentry (exhibiting a
     * sigmoidal increase of wall shear stress with increasing pressure).
     */
    QPressure mTauP;
    
    /**
     * Correction factor for TauP
     */
    QRate mkp;

public:
    
    /**
     * Constructor.
     */
    MechanicalStimulusCalculator();
    
    /**
     * Destructor.
     */
    ~MechanicalStimulusCalculator();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new class instance
     */
    static std::shared_ptr<MechanicalStimulusCalculator<DIM> > Create();

    /**
     * Return the set point shear stress as a function of pressure
     * @return the set point shear stress as a function of pressure
     */
    QPressure GetTauP();

    /**
     * Return the shear stress for very low flow rates
     * @return the shear stress for very low flow rates
     */
    QPressure GetTauReference();

    /**
     * Set the wall shear stress reference value, a lower bound for the stimulus calculation
     * @param tauRef the lower bound wall shear stress
     */
    void SetTauRef(QPressure tauRef);

    /**
     * Set the wall shear stress set point pressure value
     * @param tauP wall shear stress set point pressure value
     */
    void SetTauP(QPressure tauP);

    /**
     * Do the calculation
     */
    void Calculate();

};

#endif /* _MECHANICALSTIMULUSCALCULATOR_HPP */
