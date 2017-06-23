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

/*

 Copyright (c) 2005-2015, University of Oxford.
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

#ifndef VESSELNETWORKREADER_HPP_
#define VESSELNETWORKREADER_HPP_

#include <string>
#include "SmartPointers.hpp"
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"

/**
 * Read vessel networks from file
 */
template<unsigned DIM>
class VesselNetworkReader
{
    /**
     * The full path to the file
     */
    std::string mFileName;

    /**
     * The name of the array containing radius info
     */
    std::string mRadiusLabel;

    /**
     * Radius conversion factor. Multiply radii by this factor, useful if the contents of the
     * 'Radius' array in the vtk file is not in the form we want
     */
    double mRadiusConversionFactor;

    /**
     * Merge any coincident points in the input data
     */
    bool mMergeCoincidentPoints;

    /**
     * If this is non-zero the network will be re-sampled to a target segment length using
     * a vtk spline filter.
     */
    units::quantity<unit::length> mTargetSegmentLength;

    /**
     * The reference length scale for the vessel network, default in microns.
     */
    units::quantity<unit::length> mReferenceLength;

public:

    /**
     * Constructor
     */
    VesselNetworkReader();

    /**
     * Destructor
     */
    ~VesselNetworkReader();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     *
     * @return a shared pointer to the class instance
     */
    static std::shared_ptr<VesselNetworkReader<DIM> > Create();

    /**
     * Do the read and return the vessel network
     *
     * @return the vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > Read();

    /**
     * Set the name of the radius array
     *
     * @param rRadius the radius array name
     */
    void SetRadiusArrayName(const std::string& rRadius);

    /**
     * If true merge coincident input points
     *
     * @param mergePoints merge coincident input points
     */
    void SetMergeCoincidentPoints(bool mergePoints);

    /**
     * If nonzero resample the network to a target segment length
     *
     * @param targetSegmentLength the target segment length
     */
    void SetTargetSegmentLength(units::quantity<unit::length> targetSegmentLength);

    /**
     * Set the full path the file
     *
     * @param rFileName the full path to the file
     */
    void SetFileName(const std::string& rFileName);

    /**
     * Set the reference length scale
     *
     * @param rReferenceLength the reference length scale
     */
    void SetReferenceLengthScale(units::quantity<unit::length> rReferenceLength);
};

#endif /* VESSELNETWORKREADER_HPP_ */
