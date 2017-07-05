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

#ifndef ABSTRACTSTRUCTURALADAPTATIONSOLVER_HPP
#define ABSTRACTSTRUCTURALADAPTATIONSOLVER_HPP

#include <string>
#include <vector>
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"

/**
 * This is an abstract implementation of a structural adaptation solver. It iterates
 * until the vessel radii stop changing. Child classes implement the specific updates of the radii
 * in each iteration.
 */
template <unsigned DIM>
class AbstractStructuralAdaptationSolver
{

protected:

    /**
     *  Threshold tolerance for the maximum relative change in radii of vessels in a vessel
     *  network, below which a structural adaptation algorithm will terminate.
     */
    double mTolerance;

    /**
     *  Length of time which the algorithm is incremented on each occasion that the Iterate
     *  function is called.
     */
    QTime mTimeIncrement;

    /**
     * The reference time scale
     */
    QTime mReferenceTimeScale;

    /**
     * Whether to output the progress of the structural adaptation algorithm to a file.
     */
    bool mWriteOutput;

    /**
     * Name of file to which the progress of the structural adaptation algorithm may be
     * output.
     */
    std::string mOutputFileName;

    /**
     *  Maximum number of iterations which we may allow the algorithm to run for, regardless of timestep
     *  and SimulationTime::GetTimeStep().
     */
    unsigned mMaxIterations;

    /**
     * The vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

public:

    /**
     * Constructor
     */
    AbstractStructuralAdaptationSolver();

    /**
     * Destructor.
     */
    virtual ~AbstractStructuralAdaptationSolver();

    /**
     *  Return the tolerance for change in radii
     *  @return the tolerance for change in radii
     */
    double GetTolerance() const;

    /**
     *  Return whether the progress of the structural adaptation algorithm will be output to a
     *  file.
     *  @return whether to write to file
     */
    bool GetWriteOutput() const;

    /**
     *  Returns the filename for the file that the progress of the algorithm will be output to.
     *  @return the filename to write to
     */
    std::string GetOutputFileName() const;

    /**
     *  Returns the timeStep (in seconds).
     *  @return the time step
     */
    QTime GetTimeIncrement() const;

    /**
     *  This method should contain all of the operations used within a single iteration of a structural
     *  adaptation algorithm.
     */
    virtual void Iterate() = 0;

    /**
     *  Setter for tolerance parameter.
     *  @param tolerance the radius convergence tolerance
     */
    void SetTolerance(double tolerance);

    /**
     *  Setter for timeStep parameter.
     *  @param timeIncrement the time increment
     */
    void SetTimeIncrement(QTime timeIncrement);

    /**
     *  Setter for maximum number of iterations.
     *  @param iterations the maximum number of iterations.
     */
    void SetMaxIterations(unsigned iterations);

    /**
     *  Set whether to output the progress of structural adaptation algorithm to a file.
     *  @param writeFlag whether to write output
     */
    void SetWriteOutput(bool writeFlag);

    /**
     *  Setter for name of output file that progression of structural adaptation algorithm may
     *  be output to.
     *  @param rFilename the output filename
     */
    void SetOutputFileName(const std::string& rFilename);

    /**
     *  Set the vessel network
     *  @param pNetwork pointer to the vessel network
     */
    void SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     *  Method used to implement a structural adaptation algorithm on a vessel network.
     *  The skeleton of an algorithm is outlined in this class.  An individual iteration of an
     *  algorithm should be defined inside the Iterate method within concrete subclasses of
     *  this class.
     */
    void Solve();

    /**
     *  Method to output parameters of model to a file.  The name of the object and parameter values
     *  are appended to the file.
     */
    virtual void Write();
};

#endif // ABSTRACTSTRUCTURALADAPTATIONSOLVER_HPP
