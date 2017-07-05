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

#include <fstream>
#include <algorithm>
#include <iostream>
#include "AbstractStructuralAdaptationSolver.hpp"
#include "BaseUnits.hpp"

template<unsigned DIM>
AbstractStructuralAdaptationSolver<DIM>::AbstractStructuralAdaptationSolver()
    :   mTolerance(1.e-4),
        mTimeIncrement(1.e-4 * unit::seconds),
        mReferenceTimeScale(BaseUnits::Instance()->GetReferenceTimeScale()),
        mWriteOutput(false),
        mOutputFileName(),
        mMaxIterations(1.e5),
        mpVesselNetwork()
{

}

template<unsigned DIM>
AbstractStructuralAdaptationSolver<DIM>::~AbstractStructuralAdaptationSolver()
{

}

template<unsigned DIM>
double AbstractStructuralAdaptationSolver<DIM>::GetTolerance() const
{
    return mTolerance;
}

template<unsigned DIM>
bool AbstractStructuralAdaptationSolver<DIM>::GetWriteOutput() const
{
    return mWriteOutput;
}

template<unsigned DIM>
std::string AbstractStructuralAdaptationSolver<DIM>::GetOutputFileName() const
{
    return mOutputFileName;
}

template<unsigned DIM>
QTime AbstractStructuralAdaptationSolver<DIM>::GetTimeIncrement() const
{
    return mTimeIncrement;
}

template<unsigned DIM>
void AbstractStructuralAdaptationSolver<DIM>::SetTolerance(double tolerance)
{
    mTolerance = tolerance;
}

template<unsigned DIM>
void AbstractStructuralAdaptationSolver<DIM>::SetTimeIncrement(QTime timeIncrement)
{
    mTimeIncrement = timeIncrement;
}

template<unsigned DIM>
void AbstractStructuralAdaptationSolver<DIM>::SetMaxIterations(unsigned iterations)
{
    mMaxIterations = iterations;
}

template<unsigned DIM>
void AbstractStructuralAdaptationSolver<DIM>::SetWriteOutput(bool writeFlag)
{
    mWriteOutput = writeFlag;
}

template<unsigned DIM>
void AbstractStructuralAdaptationSolver<DIM>::SetOutputFileName(const std::string& filename)
{
    mOutputFileName = filename;
}

template<unsigned DIM>
void AbstractStructuralAdaptationSolver<DIM>::Solve()
{
    std::ofstream out;

    double max_radius_relative_change = 1.0;
    unsigned iteration = 0;
    QTime time = 0.0 * unit::seconds;

    if (mWriteOutput && !mOutputFileName.empty())
    {
        out.open(mOutputFileName.c_str());
        out << "\n";
        out << "#Iteration   Maximum relative change in radius in network\n\n";
    }

    std::vector<std::shared_ptr<VesselSegment<DIM> > > segments = mpVesselNetwork->GetVesselSegments();
    std::vector<QLength > previous_radii(segments.size());
    for (unsigned segment_index = 0; segment_index < segments.size(); segment_index++)
    {
        previous_radii[segment_index] = segments[segment_index]->GetRadius();
    }

    while (max_radius_relative_change > mTolerance && time < (SimulationTime::Instance()->GetTimeStep()*mReferenceTimeScale) && iteration < mMaxIterations)
    {
        time = time + mTimeIncrement;
        iteration++;

        Iterate();

        std::vector<double> relative_change(segments.size());
        for (unsigned segment_index = 0; segment_index < segments.size(); segment_index++)
        {
            QLength current_radius = segments[segment_index]->GetRadius();
            relative_change[segment_index] = fabs(1.0 - current_radius / previous_radii[segment_index]);
            previous_radii[segment_index] = current_radius;
        }

        max_radius_relative_change = *(std::max_element(relative_change.begin(), relative_change.end()));
        if (out.is_open())
        {
            out << std::setw(6) << iteration << std::setw(20) << max_radius_relative_change << "\n";
        }
    }
    if (out.is_open())
    {
        out.close();
    }
}

template<unsigned DIM>
void AbstractStructuralAdaptationSolver<DIM>::SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork)
{
    mpVesselNetwork = pNetwork;
}

template<unsigned DIM>
void AbstractStructuralAdaptationSolver<DIM>::Write()
{
    if(!mOutputFileName.empty())
    {
        std::ofstream out;
        out.open(mOutputFileName.c_str(), std::ios::app); // append to file
        out << "\nModule: AbstractStructuralAdaptationSolver\n";
        out << "\n---------------\n";
        out.close();
    }
}

// Explicit instantiation
template class AbstractStructuralAdaptationSolver<2> ;
template class AbstractStructuralAdaptationSolver<3> ;
