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

#include "MeshBasedCellPopulation.hpp"
#include "VolumeOutputModifier.hpp"

template<unsigned DIM>
VolumeOutputModifier<DIM>::VolumeOutputModifier()
    : AbstractCellBasedSimulationModifier<DIM>(),
      mCellLengthScale(40.0e-6), // metre
      mTimeScale(1.0), //hours
      mOutputFileStream(),
      mOutputFrequency(1)
{
}

template<unsigned DIM>
VolumeOutputModifier<DIM>::~VolumeOutputModifier()
{
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetCellLengthScale(double cellLengthScale)
{
    mCellLengthScale = cellLengthScale;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetOutputFrequency(unsigned outputFrequency)
{
    mOutputFrequency = outputFrequency;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetTimeScale(double timeScale)
{
    mTimeScale = timeScale;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetOutputFileStream(boost::shared_ptr<std::ofstream> ofstream)
{
    mOutputFileStream = ofstream;
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::UpdateAtEndOfTimeStep(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::SetupSolve(AbstractCellPopulation<DIM,DIM>& rCellPopulation, std::string outputDirectory)
{
    /*
     * We must update CellData in SetupSolve(), otherwise it will not have been
     * fully initialised by the time we enter the main time loop.
     */
    UpdateCellData(rCellPopulation);
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::UpdateCellData(AbstractCellPopulation<DIM,DIM>& rCellPopulation)
{
    // Make sure the cell population is updated
    rCellPopulation.Update();

    if(SimulationTime::Instance()->GetTimeStepsElapsed()%mOutputFrequency==0)
    {
        double total_volume = rCellPopulation.GetTetrahedralMeshForPdeModifier()->GetVolume(); // in metres

        if(mOutputFileStream->is_open())
        {
            double current_time = SimulationTime::Instance()->GetTime() * mTimeScale;
            double tumour_area = total_volume*(mCellLengthScale*mCellLengthScale);
            (*mOutputFileStream) << current_time << "," << tumour_area << "\n";
        }
    }
}

template<unsigned DIM>
void VolumeOutputModifier<DIM>::OutputSimulationModifierParameters(out_stream& rParamsFile)
{
    // No parameters to output, so just call method on direct parent class
    AbstractCellBasedSimulationModifier<DIM>::OutputSimulationModifierParameters(rParamsFile);
}

// Explicit instantiation
template class VolumeOutputModifier<1>;
template class VolumeOutputModifier<2>;
template class VolumeOutputModifier<3>;

