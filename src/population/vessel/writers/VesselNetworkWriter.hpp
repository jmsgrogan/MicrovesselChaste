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

#ifndef VESSELNETWORKWRITER_HPP_
#define VESSELNETWORKWRITER_HPP_

#include <string>
#ifdef CHASTE_VTK
#define _BACKWARD_BACKWARD_WARNING_H 1
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#endif // CHASTE_VTK
#include "VesselNetwork.hpp"

/**
 * This class converts a vessel network to a vtk polydata representation, which can be return or written to file.
 */
template<unsigned DIM>
class VesselNetworkWriter
{

private:

    /**
     * Container for the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

    /**
     * A vtk representation of the network
     */
    vtkSmartPointer<vtkPolyData> mpVtkVesselNetwork;

    /**
     * Is the current vtk representation up to date
     */
    bool mIsVtkNetworkUpToDate;

    /**
     * The output file name
     */
    std::string mFilename;

    /**
     * The reference length scale for the output.
     */
    units::quantity<unit::length> mReferenceLength;

public:

    /**
     * Constructor
     */
    VesselNetworkWriter();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a shared pointer to the class
     */
    static std::shared_ptr<VesselNetworkWriter<DIM> > Create();

    /**
     * Destructor
     */
    ~VesselNetworkWriter();

    /**
     * Set the vessel network
     * @param pNetwork the vessel network
     */
    void SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Adds a collection of vessels to the VesselNetwork
     * @param rFileName the full output path
     */
    void SetFileName(const std::string& rFileName);

    /**
     * Do the write
     */
    void Write(bool masterOnly = true);

    /**
     * Return a vtk representation of the network
     * @return the network in vtk form
     */
    vtkSmartPointer<vtkPolyData> GetOutput();

    /**
     * Set the reference length scale
     *
     * @param rReferenceLength the reference length scale
     */
    void SetReferenceLengthScale(units::quantity<unit::length> rReferenceLength);

};

#endif /* VESSELNETWORKWRITER_HPP_ */
