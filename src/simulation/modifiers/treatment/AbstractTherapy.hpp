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

#ifndef ABSTRACTTHERAPY_HPP_
#define ABSTRACTTHERAPY_HPP_

#include "UnitCollection.hpp"
#include "AbstractMicrovesselModifier.hpp"

/**
 * Abstract classes for therapies that will be applied at specified
 * time intervals and are directly related to the vasculature,
 * e.g. anti-angiogenics and chemotherapy (transported by vessels).
 */
template<unsigned DIM>
class AbstractTherapy : public AbstractMicrovesselModifier<DIM>
{

protected:

    /**
     * Amount of supplied therapy
     */
    units::quantity<unit::concentration> mAdministrationDose;

    /**
     * Times at which therapy is used
     */
    std::vector<units::quantity<unit::time> > mAdministrationTimes;

public:

    /**
     * Default constructor.
     */
    AbstractTherapy();

    /**
     * Add an administration time to the collection
     * @param time an administration time
     */
    void AddAdministrationTime(units::quantity<unit::time> time);

    /**
     * Sets doseInjected, the radiation dose injected
     * @param d dose delivered
     */
    void SetDoseInjected(units::quantity<unit::concentration> d);

    /**
     * Sets the times of administration
     * @param t the times of administration
     */
    void SetAdministrationTimes(std::vector<units::quantity<unit::time> > t);

};

#endif /*ABSTRACTTHERAPY_HPP_*/
