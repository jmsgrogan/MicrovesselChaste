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

#ifndef _RadiusCalculator_hpp
#define _RadiusCalculator_hpp

#include <boost/shared_ptr.hpp>
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkCalculator.hpp"

/**
 * This solver updates vessel radii as a result of growth stimulii according to:
 * Alarcon et al. (2003), JTB, 225, pp257-274.
 */
template<unsigned DIM>
class RadiusCalculator : public AbstractVesselNetworkCalculator<DIM>
{
    
protected:

    /**
     * The minimum allowed radius.
     */
    QLength mMinRadius;

    /**
     * The maximum allowed radius.
     */
    QLength mMaxRadius;

    /**
     * The time step for radius update
     */
    units::quantity<unit::time> mTimeStep;
    
public:
    
    /**
     * Constructor.
     */
    RadiusCalculator();
    
    /**
     * Destructor.
     */
    virtual ~RadiusCalculator();

    /**
     * Set the minimum radius
     * @param  minRadius the minimum radius
     */
    void SetMinRadius(QLength minRadius);

    /**
     * Set the maximum radius
     * @param  maxRadius the maximum radius
     */
    void SetMaxRadius(QLength maxRadius);

    /**
     * Set the time increment
     * @param  dt the time increment for radius update
     */
    void SetTimestep(units::quantity<unit::time> dt);
    
    /**
     * Do the calculation.
     */
    void Calculate();
    
};

#endif
