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

#ifndef DISCRETECONTINUUMBOUNDARYCONDITION_HPP_
#define DISCRETECONTINUUMBOUNDARYCONDITION_HPP_

#include <vector>
#include <string>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include "UblasIncludes.hpp"
#include "Part.hpp"
#include "BoundaryConditionsContainer.hpp"
#include "GridCalculator.hpp"
#include "DimensionalChastePoint.hpp"

/**
 * Helper struct for defining the type of boundary condition.
 */
struct BoundaryConditionType
{
    /**
     * The different types of boundary condition
     */
    enum Value
    {
        POINT, POLYGON, EDGE, OUTER, VESSEL_LINE, VESSEL_VOLUME, CELL, IN_PART
    };
};

/*
 * Helper struct for defining the source of the boundary condition value.
 * It can be from a data array or a single prescribed value.
 */
struct BoundaryConditionSource
{
    enum Value
    {
        LABEL_BASED, PRESCRIBED
    };
};

/*
 * A class for describing boundary conditions for use with some DiscreteContinuum solvers.
 */
template<unsigned DIM>
class DiscreteContinuumBoundaryCondition
{

protected:

    /**
     * A part for prescribing part and facet based conditions
     */
    boost::shared_ptr<Part<DIM> > mpDomain;

    /**
     * Point locations for POINT type conditions
     */
    vtkSmartPointer<vtkPoints> mpPoints;

    /**
     * The type of boundary condition
     */
    BoundaryConditionType::Value mType;

    /**
     * Where the boundary condition value is obtained from
     */
    BoundaryConditionSource::Value mSource;

    /**
     * A label specifying the array name from which to obtain the condition magnitude. Used for LABEL
     * conditions.
     */
    std::string mLabel;

    /**
     * The prescribed value of the boundary condition.
     */
    units::quantity<unit::concentration> mValue;

    /**
     * The grid calculator
     */
    boost::shared_ptr<GridCalculator<DIM> > mpGridCalculator;

    /**
     * The vessel network
     */
    boost::shared_ptr<VesselNetwork <DIM> > mpNetwork;

    /**
     * The reference concentration
     */
    units::quantity<unit::concentration> mReferenceConcentration;

    /**
     * Is this a Neumann type boundary conditions
     */
    bool mIsNeumann;

    /**
     * Is this a Robin type boundary conditions
     */
    bool mIsRobin;

public:

    /**
     * Constructor
     */
    DiscreteContinuumBoundaryCondition();

    /**
     * Destructor
     */
    virtual ~DiscreteContinuumBoundaryCondition();

    /**
     * Factory constructor method
     */
    static boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > Create();

    /**
     * Return the type of boundary condition, POINT, POLYGON, OUTER etc.
     * @return the type of boundary condition
     */
    BoundaryConditionType::Value GetType();

    /**
     * Return the default value of the boundary condition
     * @return the default value of the boundary condition
     */
    units::quantity<unit::concentration> GetValue();

    /**
     * Return the value of the boundary condition evaluated at a point and whether the point is on a boundary
     * @param location the location of the point
     * @param tolerance the tolerance for evaluating if a point is on a boundary
     * @return a bool specifying if the point is on a boundary and the value of the point on the boundary
     */
    std::pair<bool, units::quantity<unit::concentration> > GetValue(DimensionalChastePoint<DIM> location, double tolerance);

    /**
     * Update the boundary conditions container for use with the finite element solver
     * @param pContainer the boundary condition container
     */
    void UpdateBoundaryConditions(boost::shared_ptr<BoundaryConditionsContainer<DIM, DIM, 1> > pContainer);

    /**
     * Update the boundary conditions on the regular grid
     * @param pBoundaryConditions the boundary condition container
     * @param tolerance the tolerance for evaluating if a point is on a boundary
     */
    void UpdateBoundaryConditions(boost::shared_ptr<std::vector<std::pair<bool, units::quantity<unit::concentration> > > > pBoundaryConditions);

    /**
     * Set a domain for use in the calculation of FACET type boundary conditions
     * @param pDomain the part containing labelled facets for the boundary condition
     */
    void SetDomain(boost::shared_ptr<Part<DIM> > pDomain);

    /**
     * Use a robin boundary condition
     * @param isRobin use a robin boundary condition
     */
    void SetIsRobin(bool isRobin);

    /**
     * Use a neumann boundary condition
     * @param isNeumann use a neumann boundary condition
     */
    void SetIsNeumann(bool isNeumann);

    /**
     * Set the name of the label used in LABEL type sources
     * @param rLabel the label for the source strength value
     */
    void SetLabel(const std::string& label);

    /**
     * Set the points for POINT type boundary conditions
     * @param points the point locations for POINT type boundary conditions
     */
    void SetPoints(std::vector<DimensionalChastePoint<DIM> > points);

    /**
     * Set the points for POINT type boundary conditions
     * @param pPoints the point locations for POINT type boundary conditions
     */
    void SetPoints(vtkSmartPointer<vtkPoints> pPoints);

    /**
     * Set the regular grid
     * @param pGridCalculator the regular grid
     */
    void SetGridCalculator(boost::shared_ptr<GridCalculator<DIM> > pGridCalculator);

    /**
     * Set where the value of the boundary condition is obtained, e.g. LABEL, PRESCRIBED
     * @param boundarySource enum specifying where the value of the boundary condition is obtained
     */
    void SetSource(BoundaryConditionSource::Value boundarySource);

    /**
     * Set the type of boundary condition, e.g. POINT, FACET
     * @param boundaryType enum specifying the type of boundary condition
     */
    void SetType(BoundaryConditionType::Value boundaryType);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetNetwork(boost::shared_ptr<VesselNetwork <DIM> > pNetwork);

    /**
     * Set the default value of the boundary condition for any points on the boundary
     * @param value the default value of the boundary condition for any points on the boundary
     */
    void SetValue(units::quantity<unit::concentration> value);
};

#endif /* DISCRETECONTINUUMBOUNDARYCONDITION_HPP_ */
