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

#ifndef SIMPLECELLPOPULATION_HPP_
#define SIMPLECELLPOPULATION_HPP_

#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include "Part.hpp"
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "SimpleCell.hpp"
#include "UnitCollection.hpp"

/**
 * This class is used with the Python interface for generating and storing minimal cell population information.
 * It avoids some of the complexity of interfacing with full Chaste cell populations.
 */
template<unsigned DIM>
class SimpleCellPopulation
{
    /**
     * A collection of simple cells
     */
    std::vector<boost::shared_ptr<SimpleCell<DIM> > > mCells;

    /**
     * The reference length scale for the population, default in microns. This is needed as units can't be combined
     * with c_vectors, which hold the cell's location.
     */
    units::quantity<unit::length> mReferenceLength;

public:

    /**
     * Constructor
     */
    SimpleCellPopulation();

    /**
     * Factory constructor method
     *
     * @return a shared pointer to a new population
     */
    static boost::shared_ptr<SimpleCellPopulation<DIM> > Create();

    /**
     * Desctructor
     */
    ~SimpleCellPopulation();

    /**
     * Add a single SimpleCell
     *
     * @param pCell a single SimpleCell
     */
    void AddCell(boost::shared_ptr<SimpleCell<DIM> > pCell);

    /**
     * Add a vector of SimpleCells
     *
     * @param cells a vector of SimpleCells
     */
    void AddCells(std::vector<boost::shared_ptr<SimpleCell<DIM> > > cells);

    /**
     * Remove cells where they overlap a vessel network
     *
     * @param pNetwork the vessel network
     */
    void BooleanWithVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Return the collection of cells
     *
     * @return a vector  of SimpleCells
     */
    std::vector<boost::shared_ptr<SimpleCell<DIM> > > GetCells();

    /**
     * Generate cells on a regular grid
     *
     * @param xDim number of lattice points in x
     * @param yDim number of lattice points in y
     * @param zDim number of lattice points in z
     * @param spacing the grid spacing
     * @param origin the origin in the same units as the spacing
     */
    void GenerateCellsOnGrid(unsigned xDim = 10, unsigned yDim = 10, unsigned zDim = 10,
                             units::quantity<unit::length> spacing = 1.0 * 10.e-6 *unit::metres,
                             c_vector<double, DIM> origin = zero_vector<double>(DIM));

    /**
     * Generate cells on a regular grid inside a part
     *
     * @param pPart the part inside which to generate the cells
     * @param spacing the grid spacing
     */
    void GenerateCellsOnGrid(boost::shared_ptr<Part<DIM> > pPart, units::quantity<unit::length> spacing = 1.0 * 10.e-6 *unit::metres);

    /**
     * Generate cells at specifc points
     *
     * @param points a vector of points at which to generate cells
     */
    void GenerateCellsAtPoints(std::vector<c_vector<double, DIM> > points);

    /**
     * Return a vtk point representation of the cells
     *
     * @return a vtk point representation of the cells
     */
    vtkSmartPointer<vtkPoints> GetVtk();

    /**
     * Set the length scale used to dimensionalize the location as stored in mLocation.
     * @param lenthScale the reference length scale for node locations
     */
    void SetReferenceLengthScale(units::quantity<unit::length> lenthScale);

    /**
     * Write the population to file
     *
     * @param rFileName the full path to the file
     */
    void Write(const std::string& rFileName);
};

#endif /* SIMPLECELLPOPULATION_HPP_*/
