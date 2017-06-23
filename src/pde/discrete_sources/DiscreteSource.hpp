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

#ifndef DISCRETESOURCEa_HPP_
#define DISCRETESOURCEa_HPP_

#include <vector>
#include <string>
#include "AbstractDiscreteContinuumSolver.hpp"
#include "UblasIncludes.hpp"
#include "GridCalculator.hpp"
#include "UnitCollection.hpp"
#include "DensityMap.hpp"

/**
 * This class calculates the value of discrete sources at grid/mesh locations in continuum problems.
 * A grid or mesh is set and the source values are calculated at each grid point or in each element.
 * Child classes can be used to customize the way source strengths are calculated.
 */
template<unsigned DIM>
class DiscreteSource
{

protected:

    /**
     * Locations for POINT type sources
     */
    std::vector<DimensionalChastePoint<DIM> > mPoints;

    /**
     * A label specifying the array name from which to obtain the source strength. Used for LABEL
     * source strengths.
     */
    std::string mLabel;

    /**
     * The prescribed value of the source strength. Used for PRESCRIBED source strengths.
     */
    units::quantity<unit::concentration_flow_rate> mConstantInUValue;

    /**
     * The prescribed value of the source strength. Used for PRESCRIBED source strengths.
     */
    units::quantity<unit::rate> mLinearInUValue;

    /**
     * The density map
     */
    std::shared_ptr<DensityMap<DIM> > mpDensityMap;

public:

    /**
     *  Constructor
     */
    DiscreteSource();

    /**
     * Destructor
     */
    virtual ~DiscreteSource();

    /**
     * Factory constructor method
     * @return a pointer to an instance of the class
     */
    static std::shared_ptr<DiscreteSource<DIM> > Create();

    /**
     * Return the density map
     * @param pMap the density map
     */
    std::shared_ptr<DensityMap<DIM> > GetDensityMap();

    /**
     * Return the values of the source strengths sampled on the regular grid
     * @return a vector of source strengths
     */
    virtual std::vector<units::quantity<unit::concentration_flow_rate> > GetConstantInUValues();

    /**
     * Return the values of the source strengths sampled on the regular grid
     * @return a vector of source strengths
     */
    virtual std::vector<units::quantity<unit::rate> > GetLinearInUValues();

    /**
     * Return the values of the source strengths sampled on the regular grid
     * @return a vector of source strengths
     */
    virtual std::vector<units::quantity<unit::concentration_flow_rate> > GetNonlinearTermValues();

    /**
     * Set the name of the label used in LABEL type sources
     * @param rLabel the label for the source strength value
     */
    void SetLabelName(const std::string& rLabel);

    /**
     * Convencience function to get the size of arrays that will
     * be returned in advance.
     * @return the size of arrays that will be returned
     */
    unsigned GetNumberOfPoints();

    /**
     * Set the points for POINT type sources
     * @param points the point locations for POINT type sources
     */
    void SetPoints(std::vector<DimensionalChastePoint<DIM> > points);

    /**
     * Set the value of the source for PRESCRIBED type sources
     * @param value the value of the source
     */
    void SetConstantInUValue(units::quantity<unit::concentration_flow_rate> value);

    /**
     * Set the value of the source for PRESCRIBED type sources
     * @param value the value of the source
     */
    void SetLinearInUValue(units::quantity<unit::rate> value);

    /**
     * Set the density map for the calculator
     * @param pMap the density map
     */
    void SetDensityMap(std::shared_ptr<DensityMap<DIM> > pMap);

    /**
     * Update the required entries in the density map, if needed
     */
    virtual void UpdateDensityMap();
};

#endif /* DISCRETESOURCEa_HPP_ */
