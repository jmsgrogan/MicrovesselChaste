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

#include "VtkSceneMicrovesselModifier.hpp"
#include "SimulationTime.hpp"

template<unsigned DIM>
VtkSceneMicrovesselModifier<DIM>::VtkSceneMicrovesselModifier()
    : AbstractMicrovesselModifier<DIM>(),
      mpScene(),
      mUpdateFrequency(1)
{

}

template<unsigned DIM>
VtkSceneMicrovesselModifier<DIM>::~VtkSceneMicrovesselModifier()
{

}

template<unsigned DIM>
void VtkSceneMicrovesselModifier<DIM>::SetupSolve(std::string outputDirectory)
{
    if(mpScene and SimulationTime::Instance()->GetTimeStepsElapsed()%mUpdateFrequency==0)
    {
        mpScene->ResetRenderer(SimulationTime::Instance()->GetTimeStepsElapsed());
    }
}

template<unsigned DIM>
void VtkSceneMicrovesselModifier<DIM>::UpdateAtEndOfTimeStep()
{
    if(DIM>1)
    {
        if(mpScene and SimulationTime::Instance()->GetTimeStepsElapsed()%mUpdateFrequency==0)
        {
            mpScene->ResetRenderer(SimulationTime::Instance()->GetTimeStepsElapsed());
        }
    }
}

template<unsigned DIM>
std::shared_ptr<MicrovesselVtkScene<DIM> > VtkSceneMicrovesselModifier<DIM>::GetVtkScene()
{
    return mpScene;
}

template<unsigned DIM>
void VtkSceneMicrovesselModifier<DIM>::SetVtkScene(std::shared_ptr<MicrovesselVtkScene<DIM> > pScene)
{
    mpScene = pScene;
}

template<unsigned DIM>
void VtkSceneMicrovesselModifier<DIM>::SetUpdateFrequency(unsigned frequency)
{
    mUpdateFrequency = frequency;
}

// Explicit instantiation
template class VtkSceneMicrovesselModifier<2>;
template class VtkSceneMicrovesselModifier<3>;
