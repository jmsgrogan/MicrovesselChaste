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

#ifndef MICROVESSELSIMULATIONMODIFIER_HPP_
#define MICROVESSELSIMULATIONMODIFIER_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "AbstractCellBasedSimulationModifier.hpp"
#include "MicrovesselSolver.hpp"
#include "GridCalculator.hpp"

/**
 * A modifier class which at each simulation time step in a cell based simulation increments
 * a MicrovesselSolver. This solver updates a collection of PDEs and the state of
 * a vessel network. It also updates the cell data for use in the next time step of the
 * cell based simulation.
 */
template<unsigned DIM>
class MicrovesselSimulationModifier : public AbstractCellBasedSimulationModifier<DIM,DIM>
{
    /** Needed for serialization. */
    friend class boost::serialization::access;
    /**
     * Boost Serialization method for archiving/checkpointing.
     * Archives the object and its member variables.
     *
     * @param archive  The boost archive.
     * @param version  The current version of this class.
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        #if BOOST_VERSION < 105600
            EXCEPTION("Serialization not supported for Boost < 1.56")
        #else
            archive & boost::serialization::base_object<AbstractCellBasedSimulationModifier<DIM,DIM> >(*this);
            archive & mUpdateLabels;
            archive & mCellPopulationReferenceLength;
            archive & mCellPopulationReferenceConcentration;
        #endif
    }

private:

    /**
     * The vascular tumour solver
     */
    std::shared_ptr<MicrovesselSolver<DIM> > mpSolver;

    /**
     * A grid calculator for interacting with lattice based populations
     */
    std::shared_ptr<GridCalculator<DIM> > mpGridCalculator;

    /**
     * The species labels for cell data updates
     */
    std::vector<std::string> mUpdateLabels;

    /**
     * The reference length scale for the cellpopulation.
     */
    QLength mCellPopulationReferenceLength;

    /**
     * The reference concentration scale for the cellpopulation.
     */
    QConcentration mCellPopulationReferenceConcentration;

public:

    /**
     * Default constructor.
     */
    MicrovesselSimulationModifier();

    /**
     * Destructor.
     */
    virtual ~MicrovesselSimulationModifier();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a shared pointer to the class instance
     */
    static std::shared_ptr<MicrovesselSimulationModifier<DIM> > Create();

    /**
     * Overridden OutputSimulationModifierParameters() method.
     * Output any simulation modifier parameters to file.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputSimulationModifierParameters(out_stream& rParamsFile);

    /**
     * Set the labels which will be used to update cell data
     *
     * @param labels the labels which will be used to update cell data
     */
    void SetCellDataUpdateLabels(std::vector<std::string> labels);

    /**
     * Set the vascular tumour solver
     *
     * @param pSolver pointer to the vascular tumour solver
     */
    void SetMicrovesselSolver(std::shared_ptr<MicrovesselSolver<DIM> > pSolver);

    void SetGridCalculator(std::shared_ptr<GridCalculator<DIM> > pGridCalculator);

    /**
     * Overridden SetupSolve() method.
     * Specify what to do in the simulation before the start of the time loop.
     *
     * @param rCellPopulation reference to the cell population
     * @param outputDirectory the output directory, relative to where Chaste output is stored
     */
    virtual void SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory);

    /**
     * Set the length scale for the cell population
     *
     * @param cellLengthScale  the length scale for the cell population
     */
    void SetCellPopulationLengthScale(QLength cellLengthScale);

    /**
     * Set the concentration scale for the cell population
     *
     * @param cellConcentrationScale the concentration scale for the cell population
     */
    void SetCellPopulationConcentrationScale(QConcentration cellConcentrationScale);

    /**
     * Overridden UpdateAtEndOfTimeStep() method.
     * Specify what to do in the simulation at the end of each time step.
     *
     * @param rCellPopulation reference to the cell population
     */
    virtual void UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    /**
     * Helper method to compute the volume of each cell in the population and store these in the CellData.
     *
     * @param rCellPopulation reference to the cell population
     */
    void UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(MicrovesselSimulationModifier)

#endif /*MicrovesselSimulationModifier_HPP_*/
