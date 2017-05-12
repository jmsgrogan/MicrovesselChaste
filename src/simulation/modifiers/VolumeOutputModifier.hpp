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

#ifndef VOLUMEOUTPUTMODIFIER_HPP_
#define VOLUMEOUTPUTMODIFIER_HPP_

#include <fstream>
#include "SmartPointers.hpp"
#include "AbstractCellBasedSimulationModifier.hpp"
#include "UnitCollection.hpp"

/**
 * A modifier class which at each simulation time step calculates the total tumour volume
 * and writes it to file.
 */
template<unsigned DIM>
class VolumeOutputModifier : public AbstractCellBasedSimulationModifier<DIM,DIM>
{
    /**
     * The cell length scale: m
     */
    double mCellLengthScale;

    /**
     * The time scale: hours
     */
    double mTimeScale;

    /**
     * The output file stream
     */
    boost::shared_ptr<std::ofstream> mOutputFileStream;

    /**
     * The output frequency
     */
    unsigned mOutputFrequency;

    /**
     * Tumour volume only
     */
    bool mTumourVolumeOnly;

    /**
     * Only start output after this time
     */
    units::quantity<unit::time> mStartTimeOffset;

public:

    /**
     * Default constructor.
     */
    VolumeOutputModifier();

    /**
     * Destructor.
     */
    virtual ~VolumeOutputModifier();

    void SetStartTimeOffset(units::quantity<unit::time> startTimeOffset);

    void SetCellLengthScale(double cellLengthScale);

    void SetTimeScale(double timeScale);

    void SetOutputFrequency(unsigned outputFrequency);

    void SetUseTumourVolumeOnly(bool tumourVolumeOnly);

    void SetOutputFileStream(boost::shared_ptr<std::ofstream> ofstream);

    /**
     * Overridden UpdateAtEndOfTimeStep() method.
     *
     * Specify what to do in the simulation at the end of each time step.
     *
     * @param rCellPopulation reference to the cell population
     */
    virtual void UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    /**
     * Overridden SetupSolve() method.
     *
     * Specify what to do in the simulation before the start of the time loop.
     *
     * @param rCellPopulation reference to the cell population
     * @param outputDirectory the output directory, relative to where Chaste output is stored
     */
    virtual void SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory);

    /**
     * Helper method to compute the volume of each cell in the population and store these in the CellData.
     *
     * @param rCellPopulation reference to the cell population
     */
    void UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation);

    /**
     * Overridden OutputSimulationModifierParameters() method.
     * Output any simulation modifier parameters to file.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputSimulationModifierParameters(out_stream& rParamsFile);
};

#endif /*VOLUMEOUTPUTMODIFIER_HPP_*/
