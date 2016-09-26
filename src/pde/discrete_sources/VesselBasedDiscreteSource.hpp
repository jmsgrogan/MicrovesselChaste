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

#ifndef VESSELBASEDDISCRETESOURCE_HPP_
#define VESSELBASEDDISCRETESOURCE_HPP_

#include <vector>
#include <string>
#include <map>
#include "UblasIncludes.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "UnitCollection.hpp"
#include "DiscreteSource.hpp"

/**
 * This class calculates the value of discrete sources at grid/mesh locations for vessels
 */
template<unsigned DIM>
class VesselBasedDiscreteSource : public DiscreteSource<DIM>
{

protected:

    /**
     * The vessel permeability
     */
    units::quantity<unit::membrane_permeability> mVesselPermeability;

    /**
     * The reference concentration, supplied by well perfused vessels
     */
    units::quantity<unit::concentration> mReferenceConcentration;

    /**
     * The reference haematocrit, used as a proxy for vessel perfusion
     */
    units::quantity<unit::dimensionless> mReferenceHaematocrit;

public:

    /**
     * Constructor
     */
    VesselBasedDiscreteSource();

    /**
     * Destructor
     */
    virtual ~VesselBasedDiscreteSource();

    /**
     * Factory constructor method
     * @return a pointer to an instance of the class
     */
    static boost::shared_ptr<VesselBasedDiscreteSource<DIM> > Create();

    /**
     * Return the values of the source strengths sampled on the mesh elements
     * @return a vector of source strengths
     */
    std::vector<units::quantity<unit::concentration_flow_rate> > GetConstantInUMeshValues();

    /**
     * Return the values of the source strengths sampled on the mesh elements
     * @return a vector of source strengths
     */
    std::vector<units::quantity<unit::rate> > GetLinearInUMeshValues();

    /**
     * Return the values of the source strengths sampled on the regular grid
     * @return a vector of source strengths
     */
    std::vector<units::quantity<unit::concentration_flow_rate> > GetConstantInURegularGridValues();

    /**
     * Return the values of the source strengths sampled on the regular grid
     * @return a vector of source strengths
     */
    std::vector<units::quantity<unit::rate> > GetLinearInURegularGridValues();

    /**
     * Set the value of the source for PRESCRIBED type sources
     * @param value the value of the source
     */
    void SetVesselPermeability(units::quantity<unit::membrane_permeability> value);

    /**
     * Set the value of the source for PRESCRIBED type sources
     * @param value the value of the source
     */
    void SetReferenceConcentration(units::quantity<unit::concentration> value);

    /**
     * Set the value of the source for PRESCRIBED type sources
     * @param value the value of the source
     */
    void SetReferenceHaematocrit(units::quantity<unit::dimensionless> value);

};

#endif /* VESSELBASEDDISCRETESOURCE_HPP_ */
