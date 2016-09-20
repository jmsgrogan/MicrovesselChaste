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

#ifndef ABSTRACTUNSTRUCTUREDGRIDDISCRETECONTINUUMSOLVER_HPP_
#define ABSTRACTUNSTRUCTUREDGRIDDISCRETECONTINUUMSOLVER_HPP_

#include <vector>
#include <string>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkUnstructuredGrid.h>
#include <vtkSmartPointer.h>
#include "SmartPointers.hpp"
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
    using AbstractDiscreteContinuumSolver<DIM>::GetConcentrations;
    using AbstractDiscreteContinuumSolver<DIM>::GetSolution;
    using AbstractDiscreteContinuumSolver<DIM>::UpdateSolution;

protected:

    /**
     * The solution in VTK form
     */
    vtkSmartPointer<vtkUnstructuredGrid> mpVtkSolution;

    /**
     * The finite element mesh
     */
    boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > mpMesh;

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
     * Return the mesh
     * @return a pointer to the mesh
     */
    boost::shared_ptr<DiscreteContinuumMesh<DIM> > GetMesh();

    /**
     * Return the value of the field at the requested points
     * @param rSamplePoints a vector of sample points
     * @return the value of the field ordered according to input point order
     */
    virtual std::vector<units::quantity<unit::concentration> > GetConcentrations(const std::vector<DimensionalChastePoint<DIM> >& samplePoints);
    /**
     * Return the value of the field at the requested points
     * @param pGrid the sampling grid
     * @return the value of the field ordered according to grid order
     */
    virtual std::vector<units::quantity<unit::concentration> > GetConcentrations(boost::shared_ptr<RegularGrid<DIM> > pGrid);

    /**
     * Return the value of the field on the nodes of the input mesh
     * @param pMesh the mesh from which nodes are sampled
     * @return the value of the field ordered according to mesh node ordering
     */
    virtual std::vector<units::quantity<unit::concentration> > GetConcentrations(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh);

    /**
     * Return the value of the field at the mesh centroids
     * @return the value of the field ordered according to element order
     */
    virtual std::vector<units::quantity<unit::concentration> > GetConcentrationsAtCentroids();

    /**
     * Return the value of the field at the requested points
     * @param rSamplePoints the points for sampling
     * @return the value of the field ordered according to input point order
     */
    virtual std::vector<double> GetSolution(const std::vector<DimensionalChastePoint<DIM> >& rSamplePoints);

    /**
     * Return the value of the field at all points on the supplied grid
     * @param pGrid the grid to be sampled
     * @return the value of the field ordered according to input point order
     */
    virtual std::vector<double> GetSolution(boost::shared_ptr<RegularGrid<DIM> > pGrid);

    /**
     * Return the value of the field at all points on the supplied mesh nodes
     * @param pMesh the mesh for point sampling
     * @return the value of the field ordered according to mesh node order
     */
    virtual std::vector<double> GetSolution(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh);

    /**
     * Return the solution as vtk image data
     */
    virtual vtkSmartPointer<vtkUnstructuredGrid> GetVtkSolution();

    /**
     * Set the mesh
     * @param pMesh the finite element mesh
     */
    void SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM, DIM> > pMesh);

    /**
     * Overridden Setup method.
     */
    virtual void Setup();

    virtual void UpdateSolution(const std::vector<double>& data);

    virtual void UpdateElementSolution(const std::vector<double>& data);

    virtual void UpdateSolution(const std::vector<units::quantity<unit::concentration> >& data);

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
