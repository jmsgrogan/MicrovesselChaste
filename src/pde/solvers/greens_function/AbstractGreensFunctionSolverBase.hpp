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



#ifndef ABSTRACTGREENSFUNCTIONSOLVERBASE_HPP_
#define ABSTRACTGREENSFUNCTIONSOLVERBASE_HPP_

#include <vector>
#include <string>
#include <map>
#include <boost/multi_array.hpp>
#include "ChastePoint.hpp"
#include "SmartPointers.hpp"
#include "VesselSegment.hpp"
#include "Part.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"
#include "UnitCollection.hpp"

/**
 * A class for solving diffusion-reaction PDEs based on the Greens Function method of Secomb and co-workers.
 *
 * The method is described here:
 * Secomb, T.W., Hsu, R., Park, E.Y.H. and Dewhirst, M.W. Green's function methods for analysis of oxygen delivery to tissue by microvascular networks.
 * Annals of Biomedical Engineering, 32: 1519-1529 (2004).
 *
 * The present code is based on a C++ implementation available here (without license):
 * http://physiology.arizona.edu/people/secomb/greens_c3
 * however it has been re-written from scratch following Chaste style and using PETSc linear solvers.
 */
template<unsigned DIM>
class AbstractGreensFunctionSolverBase : public AbstractRegularGridDiscreteContinuumSolver<DIM>
{

protected:

    /**
     * A cuboidal tissue domain
     */
    std::shared_ptr<Part<DIM> > mpDomain;

    /**
     * The positions of point sinks
     */
    std::vector<DimensionalChastePoint<DIM> > mSinkCoordinates;

    /**
     * Map between point sinks and segments
     */
    std::vector<unsigned> mSinkPointMap;

    /**
     * Coorindates of subsegments
     */
    std::vector<DimensionalChastePoint<DIM> > mSubSegmentCoordinates;

    /**
     * Subsegment lengths
     */
    std::vector<QLength > mSubSegmentLengths;

    /**
     * Sink rate ordered by point index
     */
    std::vector<QMolarFlowRate > mSinkRates;

    /**
     * Source rate ordered by point index
     */
    std::vector<QMolarFlowRate > mSourceRates;

    /**
     * Species concentration in vessels
     */
    std::vector<QConcentration > mSegmentConcentration;

    /**
     * Map between vessel segments and point locations
     */
    std::map<unsigned, std::shared_ptr<VesselSegment<DIM> > > mSegmentPointMap;

    /**
     * Greens function matrix for tissue-tissue case
     */
    std::shared_ptr<boost::multi_array<QPerLength , 2> > mGtt;

    /**
     * Greens function matrix for vessel-vessel case
     */
    std::shared_ptr<boost::multi_array<QPerLength , 2> > mGvv;

    /**
     * Greens function matrix for vessel-tissue case
     */
    std::shared_ptr<boost::multi_array<QPerLength , 2> > mGvt;

    /**
     * Greens function matrix for tissue-vessel case
     */
    std::shared_ptr<boost::multi_array<QPerLength , 2> > mGtv;

    /**
     * Minimum subsegment length
     */
    QLength mSubsegmentCutoff;

public:

    /**
     * Constructor
     */
    AbstractGreensFunctionSolverBase();

    /**
     * Destructor
     */
    virtual ~AbstractGreensFunctionSolverBase();

    /**
     * Generate vessel subsegments
     */
    void GenerateSubSegments();

    /**
     * Generate tissue points
     */
    void GenerateTissuePoints();

    /**
     * Update Gvv
     * @return Gvv
     */
    virtual std::shared_ptr<boost::multi_array<QPerLength, 2> > GetVesselVesselInteractionMatrix();

    /**
     * Update Gtt
     * @return Gtt
     */
    virtual std::shared_ptr<boost::multi_array<QPerLength, 2> > GetTissueTissueInteractionMatrix();

    /**
     * Update Gtv
     * @return Gtv
     */
    virtual std::shared_ptr<boost::multi_array<QPerLength, 2> > GetTissueVesselInteractionMatrix();

    /**
     * Update Gvt
     * @return Gvt
     */
    virtual std::shared_ptr<boost::multi_array<QPerLength, 2> > GetVesselTissueInteractionMatrix();

    /**
     * Set the minimum subsegment length
     * @param value the minimum subsegment length
     */
    void SetSubSegmentCutoff(QLength value);

    /**
     * Do the solve
     */
    virtual void Solve()=0;

    /**
     * Update the greens function matrices
     * @param updateGtt Update Gtt
     * @param updateGvv Update Gvv
     * @param updateGtv Update Gtv
     * @param updateGvt Update Gvt
     */
    virtual void UpdateGreensFunctionMatrices(bool updateGtt = 0, bool updateGvv = 0, bool updateGtv = 0, bool updateGvt = 0);

    /**
     * Over-ridden method for writing solution to file
     * @param segmentPointData the concentrations in each segment
     */
    void WriteSolution(std::map<std::string, std::vector<QConcentration > >& segmentPointData);
};

#endif /* ABSTRACTGREENSFUNCTIONSOLVERBASE_HPP_ */
