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

#ifndef CELLSTATEDEPENDENTDISCRETESOURCE_HPP_
#define CELLSTATEDEPENDENTDISCRETESOURCE_HPP_

#include <vector>
#include <string>
#include <map>
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "AbstractCellProperty.hpp"
#include "ApoptoticCellProperty.hpp"
#include "DiscreteSource.hpp"
#include "UnitCollection.hpp"

/**
 * Specialization of discrete sources for cell state specific sink and source strengths
 */
template<unsigned DIM>
class CellStateDependentDiscreteSource : public DiscreteSource<DIM>
{

private:

    /**
     * Cells of different 'color' (i.e. mutation state label) can have different sink rates.
     */
    std::map<unsigned, units::quantity<unit::concentration_flow_rate> > mStateRateMap;

    /**
     * Cells of different 'color' (i.e. mutation state label) can have different sink rate thresholds.
     */
    std::map<unsigned, units::quantity<unit::concentration> > mStateRateThresholdMap;

    /**
     * The consumption rate per unit species concentration
     */
    units::quantity<unit::flow_rate> mConsumptionRatePerUnitConcentration;

public:

    /**
     *  Constructor
     */
    CellStateDependentDiscreteSource();

    /**
     * Destructor
     */
    virtual ~CellStateDependentDiscreteSource();

    /**
     * Factory constructor method
     * @return a pointer to an instance of the class
     */
    static boost::shared_ptr<CellStateDependentDiscreteSource<DIM> > Create();

    /**
     * Return the values of the source strengths sampled on the regular grid
     * @return a vector of source strengths
     */
    std::vector<units::quantity<unit::concentration_flow_rate> > GetConstantInUValues();

    /**
     * Return the values of the source strengths sampled on the regular grid
     * @return a vector of source strengths
     */
    std::vector<units::quantity<unit::rate> > GetLinearInUValues();

    /**
     * Set cell 'color' specific consumption rates. 'Color' is a property of the cell mutation state set in its constructor.
     * It is up to the user to ensure that different mutation states of interest have different 'color' values assigned.
     * If a cell 'color' key is requested from the map that does not have a value assigned then its consumption rate will
     * be 0.0.
     *
     * @param stateRateMap the label for the source strength value
     */
    void SetStateRateMap(std::map<unsigned, units::quantity<unit::concentration_flow_rate> > stateRateMap);

    /**
     * Set cell 'color' specific consumption rate thresholds.
     * If a cell 'color' key is requested from the map that does not have a value assigned then its consumption rate threshold will
     * be 0.0.
     *
     * @param stateThresholdMap the label for the source strength value
     */
    void SetStateRateThresholdMap(std::map<unsigned, units::quantity<unit::concentration> > stateThresholdMap);
};

#endif /* CELLSTATEDEPENDENTDISCRETESOURCE_HPP_ */
