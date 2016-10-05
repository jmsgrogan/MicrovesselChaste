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

#ifndef ABSTRACTDISCRETECONTINUUMSOLVER_HPP_
#define ABSTRACTDISCRETECONTINUUMSOLVER_HPP_

#include <vector>
#include <string>
#include "OutputFileHandler.hpp"
#include "VesselNetwork.hpp"
#include "AbstractCellPopulation.hpp"
#include "AbstractDiscreteContinuumLinearEllipticPde.hpp"
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"
#include "DiscreteContinuumBoundaryCondition.hpp"
#include "RegularGrid.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "UnitCollection.hpp"

/**
 * An abstract solver class for continuum-discrete field problems.
 * The class is used by the MicrovesselSolver to provide a concentration or dimensionless field for a single,
 * labelled quantity for cells and/or vessels. It contains methods for sampling on structured and unstructured grids.
 * Derived classes are responsible for updating the values of data fields in cells
 * and vessels on each call and optionally writing the solution to file.
 */
template<unsigned DIM>
class AbstractDiscreteContinuumSolver
{

protected:

    /**
     * The vessel network.
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The cell population.
     */
    AbstractCellPopulation<DIM>* mpCellPopulation;

    /**
     * File handler containing the output directory
     */
    boost::shared_ptr<OutputFileHandler> mpOutputFileHandler;

    /**
     *  The filename for output
     */
    std::string mFilename;

    /**
     *  The label for the quantity being solved for
     */
    std::string mLabel;

    /**
     *  Has the Setup function been called.
     */
    bool IsSetupForSolve;

    /**
     *  Should the solution be written to file
     */
    bool mWriteSolution;

    /**
     * The PDE to be solved
     */
    boost::shared_ptr<AbstractDiscreteContinuumLinearEllipticPde<DIM, DIM> > mpPde;

    /**
     * The non-linear PDE to be solved
     */
    boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > mpNonLinearPde;

    /**
     * The DiscreteContinuum boundary conditions, optional
     */
    std::vector<boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > > mBoundaryConditions;

    /**
     * This is used internally to scale concentrations before and after linear system solves, reads and writes.
     * Since those functions don't use Boost Units. It should not affect the solution, but can be judiciously chosen
     * to avoid precision problems.
     */
    units::quantity<unit::concentration> mReferenceConcentration;

    /**
     * A solution field. Ordering is decided in child classes.
     */
    std::vector<double> mSolution;

    /**
     * A solution field. Ordering is decided in child classes.
     */
    std::vector<units::quantity<unit::concentration> > mConcentrations;

    /**
     * Used to check the type of solver
     */
    bool mHasRegularGrid;

    /**
     * Used to check the type of solver
     */
    bool mHasUnstructuredGrid;

public:

    /**
     * Constructor
     */
    AbstractDiscreteContinuumSolver();

    /**
     * Destructor
     */
    virtual ~AbstractDiscreteContinuumSolver();

    /**
     * Add a DiscreteContinuum boundary condition for the domain
     * @param pBoundaryCondition the boundary condition
     */
    void AddBoundaryCondition(boost::shared_ptr<DiscreteContinuumBoundaryCondition<DIM> > pBoundaryCondition);

    /**
     * Has a cell population been set?
     * @return whether a cell population been set.
     */
    bool CellPopulationIsSet();

    /**
     * Return the value of the field with ordering determined by child classes
     * @return the value of the field with ordering determined by child classes
     */
    virtual std::vector<units::quantity<unit::concentration> > GetConcentrations();

    /**
     * Return the value of the field at all points on the supplied grid
     * @param pGrid the sampling grid
     * @return the value of the field ordered according to grid order
     */
    virtual std::vector<units::quantity<unit::concentration> > GetConcentrations(boost::shared_ptr<RegularGrid<DIM> > pGrid) = 0;

    /**
     * Return the value of the field at the requested points
     * @param rSamplePoints a vector of sample points
     * @return the value of the field ordered according to input point order
     */
    virtual std::vector<units::quantity<unit::concentration> > GetConcentrations(const std::vector<DimensionalChastePoint<DIM> >& rSamplePoints) = 0;

    /**
     * Return the value of the field on the nodes of the input mesh
     * @param pMesh the mesh from which nodes are sampled
     * @return the value of the field ordered according to mesh node ordering
     */
    virtual std::vector<units::quantity<unit::concentration> > GetConcentrations(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh) = 0;

    /**
     * Return the name of the field being solved for
     * @return a reference to the field name
     */
    const std::string& GetLabel();

    /**
     * Return the nonlinear PDE
     * @return the DiscreteContinuum nonlinear elliptic pde
     */
    boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > GetNonLinearPde();

    /**
     * Return the PDE
     * @return the DiscreteContinuum linear elliptic pde
     */
    boost::shared_ptr<AbstractDiscreteContinuumLinearEllipticPde<DIM, DIM> > GetPde();

    /**
     * Return the reference concentration value.
     * @return the reference concentration value
     */
    units::quantity<unit::concentration> GetReferenceConcentration();

    /**
     * Return the value of the field with ordering determined by child classes
     * @return the value of the field with ordering determined by child classes
     */
    virtual std::vector<double> GetSolution();

    /**
     * Return the value of the field at the requested points
     * @param rSamplePoints the points for sampling
     * @return the value of the field ordered according to input point order
     */
    virtual std::vector<double> GetSolution(const std::vector<DimensionalChastePoint<DIM> >& rSamplePoints) = 0;

    /**
     * Return the value of the field at all points on the supplied grid
     * @param pGrid the grid to be sampled
     * @return the value of the field ordered according to input point order
     */
    virtual std::vector<double> GetSolution(boost::shared_ptr<RegularGrid<DIM> > pGrid) = 0;

    /**
     * Return the value of the field at all points on the supplied mesh nodes
     * @param pMesh the mesh for point sampling
     * @return the value of the field ordered according to mesh node order
     */
    virtual std::vector<double> GetSolution(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh) = 0;

    /**
     * Return true if the solver uses a regular grid to store solutions
     * @return true if the solver uses a regular grid to store solutions
     */
    bool HasRegularGrid();

    /**
     * Return true if the solver uses a unstructured grid to store solutions
     * @return true if the solver uses a unstructured grid to store solutions
     */
    bool HasUnstructuredGrid();

    /**
     * Set the cell population
     * @param rCellPopulation a reference to the cell population
     */
    void SetCellPopulation(AbstractCellPopulation<DIM>& rCellPopulation);

    /**
     * Set the file handler containing the working directory
     * @param pOutputFileHandler the file handler containing the working directory
     */
    void SetFileHandler(boost::shared_ptr<OutputFileHandler> pOutputFileHandler);

    /**
     * Set the file name for output
     * @param rFilename the file name
     */
    void SetFileName(const std::string& rFilename);

    /**
     * Set the name of the field being solved for
     * @param rLabel a reference to the field name
     */
    void SetLabel(const std::string& rLabel);

    /**
     *  Set the PDE to be solved
     * @param pPde the pde to be solved
     */
    void SetPde(boost::shared_ptr<AbstractDiscreteContinuumLinearEllipticPde<DIM, DIM> > pPde);

    /**
     *  Set the nonlinear PDE to be solved
     * @param pPde the pde to be solved
     */
    void SetNonLinearPde(boost::shared_ptr<AbstractDiscreteContinuumNonLinearEllipticPde<DIM, DIM> > pPde);

    /**
     * Operations to be performed prior to the first solve
     */
    virtual void Setup() = 0;

    /**
     * Set the reference concentration
     * @param referenceConcentration the reference concentration
     */
    void SetReferenceConcentration(units::quantity<unit::concentration> referenceConcentration);

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set whether to write the solution to file on next solve
     * @param write write the solution
     */
    void SetWriteSolution(bool write=true);

    /**
     * Do the solve
     */
    virtual void Solve() = 0;

    /**
     * Operations to be performed prior to every solve
     */
    virtual void Update() = 0;

    /**
     * Set the cell data to the values in the field
     */
    virtual void UpdateCellData() = 0;

    /**
     * Update the solution manually
     * @param rData solution data map
     */
    virtual void UpdateSolution(const std::vector<double>& rData);

    /**
     * Update the solution manually
     * @param rData solution data map
     */
    virtual void UpdateSolution(const std::vector<units::quantity<unit::concentration> >& rData);

    /**
     * Write the solution to file
     */
    virtual void Write() = 0;
};

#endif /* ABSTRACTDISCRETECONTINUUMSOLVER_HPP_ */
