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

#ifndef _VISCOSITYCALCULATOR_HPP
#define _VISCOSITYCALCULATOR_HPP

#include <boost/shared_ptr.hpp>
#include "AbstractVesselNetworkCalculator.hpp"

/**
 * This solver calculates the dynamic viscosity in a vessel as a function of radius and haematocrit according to:
 * Alarcon et al. (2003), JTB, 225, pp257-274.
 */
template<unsigned DIM>
class ViscosityCalculator : public AbstractVesselNetworkCalculator<DIM>
{
    
    /**
     * The plasma viscosity.
     */
    units::quantity<unit::dynamic_viscosity> mPlasmaViscosity;

public:
    
    /**
     * Constructor.
     */
    ViscosityCalculator();

    /**
     *  Destructor.
     */
    ~ViscosityCalculator();
    
    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new class instance
     */
    static boost::shared_ptr<ViscosityCalculator<DIM> > Create();

    /**
     * Do the calculation.
     */
    void Calculate();

    /**
     * Set the plasma viscosity
     * @param viscosity the plasma viscosity
     */
    void SetPlasmaViscosity(units::quantity<unit::dynamic_viscosity> viscosity);
};

#endif
