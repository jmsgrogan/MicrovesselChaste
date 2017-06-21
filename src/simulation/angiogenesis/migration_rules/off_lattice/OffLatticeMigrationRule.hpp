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

#ifndef OFFLATTICEMIGRATIONRULE_HPP_
#define OFFLATTICEMIGRATIONRULE_HPP_

#include <vector>
#include <string>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include "AbstractMigrationRule.hpp"
#include "VesselNode.hpp"
#include "SmartPointers.hpp"

/**
 * An off-lattice migration rule for tip cells
 */
template<unsigned DIM>
class OffLatticeMigrationRule : public AbstractMigrationRule<DIM>
{
    /**
     * Global direction vectors, x (1,0,0)
     */
    c_vector<double, 3> mGlobalX;

    /**
     * Global direction vectors, y (0,1,0)
     */
    c_vector<double, 3> mGlobalY;

    /**
     * Global direction vectors, z (0,0,1)
     */
    c_vector<double, 3> mGlobalZ;

    /**
     * Mean angle between current and new directions about global axes
     */
    std::vector<units::quantity<unit::plane_angle> > mMeanAngles;

    /**
     * Deviation in angle between current and new directions about global axes
     */
    std::vector<units::quantity<unit::plane_angle> > mSdvAngles;

    /**
     * Tip cell velocity
     */
    units::quantity<unit::velocity> mVelocity;

    /**
     * Chemotactic strength
     */
    double mChemotacticStrength;

    /**
     * Vessel-vessel attraction strength
     */
    double mAttractionStrength;

    /**
     * Length of probe into solution
     */
    units::quantity<unit::length> mProbeLength;

    /**
     * Length beyond which there is no mutual attraction
     */
    units::quantity<unit::length> mCriticalMutualAttractionLength;

    /**
     * Add a surface repulsion
     */
    bool mSurfaceRepulsion;

    /**
     * Controls number of sample points used in gradient evaluation
     */
    unsigned mNumGradientEvaluationDivisions;

    /**
     * Distance map for repulsion evaluation
     */
    vtkSmartPointer<vtkDataSet> mpDomainDistanceMap;

public:

    /**
     * Constructor.
     */
    OffLatticeMigrationRule();

    /**
     * Destructor.
     */
    virtual ~OffLatticeMigrationRule();

    /**
     * Calculate the distance map for repulsion evaluation. This version
     * is used when no solver is specified
     */
    void CalculateDomainDistanceMap();

    /**
     * Calculate the distance map for repulsion evaluation on the supplied grid.
     * @param pGrid get the distance map on this grid
     */
    void CalculateDomainDistanceMap(boost::shared_ptr<AbstractDiscreteContinuumGrid<DIM> > pGrid);

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return pointer to a new class instance
     */
    static boost::shared_ptr<OffLatticeMigrationRule<DIM> > Create();

    /**
     * Return the movement vector (new_location - oriringal_location) for the input nodes, if they can't move set it to the zero vector
     * @param rNodes nodes to calculate indices
     * @return a vector of movement vectors
     */
    std::vector<DimensionalChastePoint<DIM> > GetDirections(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Get the sprout directions
     * @param rNodes nodes to calculate directions
     * @return a vector of movement vectors
     */
    std::vector<DimensionalChastePoint<DIM> > GetDirectionsForSprouts(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Set the sprout velocity
     * @param velocity the sprout velocity
     */
    void SetSproutingVelocity(units::quantity<unit::velocity> velocity);

    /**
     * Set the chemotactic strength
     * @param strength the chemotactic strength
     */
    void SetChemotacticStrength(double strength);

    /**
     * Set the mutual attraction strength
     * @param strength the mutual attraction strength
     */
    void SetAttractionStrength(double strength);

    /**
     * Set number of sample points used in gradient evaluation. In 2D it
     * should be an even number.
     * @param numDivisions number of sample points used in gradient evaluation
     */
    void SetNumGradientEvaluationDivisions(unsigned numDivisions);

    /**
     * Set the standard deviation of the persistence angle
     * @param angle the standard deviation of the persistence angle
     */
    void SetPersistenceAngleSdv(double angle);

};

#endif /* OffLatticeMigrationRule_HPP_ */
