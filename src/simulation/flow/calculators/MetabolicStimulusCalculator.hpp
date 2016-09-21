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

#ifndef _METABOLICSTIMULUSCALCULATOR_HPP
#define _METABOLICSTIMULUSCALCULATOR_HPP

#include "SmartPointers.hpp"
#include "AbstractVesselNetworkCalculator.hpp"
#include "UnitCollection.hpp"

/**
 * Calculate a stimulus related to the ability of the vasculature to adopt to the
 * metabolic need of the tissue. See Alarcon et al. (2003), JTB, 225, pp257-274.
 */
template<unsigned DIM>
class MetabolicStimulusCalculator  : public AbstractVesselNetworkCalculator<DIM>
{

private:

    /**
     * Reference flow rate
     */
    units::quantity<unit::flow_rate> mQRef;

    /**
     * Metabolic Stimulus Constant
     */
    units::quantity<unit::rate> mKm;

    /**
     * Maximum stimulus
     */
    units::quantity<unit::rate> mMaxStimulus;

public:

    /**
     * Constructor.
     */
    MetabolicStimulusCalculator();

    /**
     * Destructor.
     */
    ~MetabolicStimulusCalculator();

    /**
     * Get the reference flow rate
     * @return reference flow rate
     */
    units::quantity<unit::flow_rate> GetQRef();

    /**
     * Get the stimulus constant
     * @return the stimulus constant
     */
    units::quantity<unit::rate> GetKm();

    /**
     * Get the maximum stimulus
     * @return the maximum stimulus
     */
    units::quantity<unit::rate> GetMaxStimulus();

    /**
     * set the reference flow rate
     * @param qRef reference flow rate
     */
    void SetQRef(units::quantity<unit::flow_rate> qRef);

    /**
     * set the stimulus constant
     * @param km stimulus constant
     */
    void SetKm(units::quantity<unit::rate> km);

    /**
     * set the maximum stimulus
     * @param maxStimulus the maximum stimulus
     */
    void SetMaxStimulus(units::quantity<unit::rate> maxStimulus);

    /**
     * Do the calculation
     */
    void Calculate();

};

#endif
