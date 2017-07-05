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

#ifndef ABSTRACTUNSTRUCTUREDGRIDDISCRETECONTINUUMSOLVER_HPP_
#define ABSTRACTUNSTRUCTUREDGRIDDISCRETECONTINUUMSOLVER_HPP_

#include <memory>
#include <vector>
#include <string>
#include "UblasIncludes.hpp"
#include "AbstractDiscreteContinuumSolver.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "UnitCollection.hpp"

/**
 * An abstract solver class for DiscreteContinuum continuum-discrete problems using structured grids.
 * Concrete classes can solve PDEs or perform other computations based on interpolation
 * of discrete entities (points/cells, lines/vessels) onto structured grids.
 */
template<unsigned DIM>
class AbstractUnstructuredGridDiscreteContinuumSolver : public AbstractDiscreteContinuumSolver<DIM>
{
    using AbstractDiscreteContinuumSolver<DIM>::UpdateSolution;

protected:

    /**
     * The finite element mesh
     */
    std::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > mpMesh;

public:

    /**
     * Constructor
     */
    AbstractUnstructuredGridDiscreteContinuumSolver();

    /**
     * Destructor
     */
    virtual ~AbstractUnstructuredGridDiscreteContinuumSolver();

    /**
     * Return the value of the field at the mesh centroids
     * @return the value of the field ordered according to element order
     */
    virtual std::vector<QConcentration > GetConcentrationsAtCentroids();

    /**
     * Overridden Setup method.
     */
    virtual void Setup();

    /**
     * Update the solution using dimensionless data
     * @param rData the data
     */
    virtual void UpdateSolution(const std::vector<double>& rData);

    /**
     * Update the element solution using dimensionless data
     * @param rData the data
     */
    virtual void UpdateElementSolution(const std::vector<double>& rData);

    /**
     * Update the solution using concentration data
     * @param rData the data
     */
    virtual void UpdateSolution(const std::vector<QConcentration >& rData);

    /**
     * Update the cell data as passed in
     */
    virtual void UpdateCellData();

    /**
     * Overridden Update method.
     */
    virtual void Update();

    /**
     * Overridden Update method.
     */
    virtual void Solve() = 0;

    /**
     * Overridden Write method. Writes the solution to file
     */
    virtual void Write();
};

#endif /* ABSTRACTUNSTRUCTUREDGRIDDISCRETECONTINUUMSOLVER_HPP_ */
