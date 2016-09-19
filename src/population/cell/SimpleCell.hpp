/*

 Copyright (c) 2005-2015, University of Oxford.
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

#ifndef SIMPLECELL_HPP_
#define SIMPLECELL_HPP_

#include "ChastePoint.hpp"
#include "UblasIncludes.hpp"
#include "SmartPointers.hpp"
#include "AbstractCellProperty.hpp"
#include "UnitCollection.hpp"

/**
 * A minimal cell class used to set up Chaste cell populations. It is particularly useful in combination with
 * the Python interface.
 */
template<unsigned DIM>
class SimpleCell : public ChastePoint<DIM>
{
    /**
     * An index for the cell
     */
    unsigned mIndex;

    /**
     * The name of the cell cycle model
     */
    std::string mCellCycleModel;

    /**
     * The name of the cell mutation state
     */
    std::string mCellMutationState;

    /**
     * The name of the cell proliferative type
     */
    std::string mCellProliferativeType;

    /**
     * The reference length scale for the cell, default in microns. This is needed as units can't be combined
     * with c_vectors, which hold the cell's location.
     */
    units::quantity<unit::length> mReferenceLength;

public:

    /**
     * Constructor
     *
     * @param v1 x position
     * @param v2 y position
     * @param v3 z position
     */
    SimpleCell(double v1 = 0, double v2 = 0, double v3 = 0);

    /**
     * Constructor
     *
     * @param location the location of the cell
     */
    SimpleCell(c_vector<double, DIM> location);

    /**
     * Factory constructor method
     *
     * @param v1 x position
     * @param v2 y position
     * @param v3 z position
     * @return a shared pointer to a new cell
     */
    static boost::shared_ptr<SimpleCell<DIM> > Create(double v1 = 0, double v2 = 0, double v3 = 0);

    /**
     * Factory constructor method
     *
     * @param location the location of the cell
     *
     * @return a shared pointer to a new cell
     */
    static boost::shared_ptr<SimpleCell<DIM> > Create(c_vector<double, DIM> location);

    /**
     * Desctructor
     */
    ~SimpleCell();

    /**
     * Return the cell index
     *
     * @return cell index
     */
    unsigned GetIndex();

    /**
     * Return the reference length scale for the cell, default is micron
     *
     */
    units::quantity<unit::length> GetReferenceLengthScale() const;

    /**
     * Set the cell index
     *
     * @param index the cell index
     */
    void SetIndex(unsigned index);

    /**
     * Set the length scale used to dimensionalize the location as stored in mLocation.
     * @param lenthScale the reference length scale for node locations
     */
    void SetReferenceLengthScale(units::quantity<unit::length> lenthScale);
};

#endif /* SIMPLECELL_HPP_*/
