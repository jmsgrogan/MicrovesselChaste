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

#ifndef NETWORKTOSURFACE_HPP_
#define NETWORKTOSURFACE_HPP_

#include <memory>
#include <vector>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkPolyData.h>
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include <boost/units/quantity.hpp>
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"
#include "NetworkToImage.hpp"

/**
 * This class generates a 2d or 3d triangulated surface from a vessel network (centrelines and radii).
 * It works by sampling the network onto a regular grid, with points inside and outside vessels having different
 * values. It then reconstructs and optionally smooths the surface. The surface does not necessarily have high
 * quality triangles in 3d, they can be added using other tools such as the NetworkToSurface class.
 */
template<unsigned DIM>
class NetworkToSurface
{
    /**
     * The network to image tool
     */
    std::shared_ptr<NetworkToImage<DIM> > mpNetworkToImage;

    /**
     * The vessel network
     */
    std::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The length for spline re-sampling
     */
    units::quantity<unit::length> mSplineResamplingLength;

    /**
     * The surface
     */
    vtkSmartPointer<vtkPolyData> mpSurface;

    /**
     * The reference length scale
     */
    units::quantity<unit::length> mReferenceLength;

    /**
     * Whether to do smoothing
     */
    bool mDoSmoothing;

    /**
     * Smoothing Iterations
     */
    unsigned mNumSmoothingIterations;

    /**
     * Feature angle
     */
    double mSmoothingFeatureAngle;

    /**
     * Band pass frequency
     */
    double mBandPassFrequency;

    /**
     * Target mesh length
     */
    double mTargetMeshLength;

    /**
     * Whether to do surface remeshing
     */
    bool mDoSurfaceRemeshing;

public:

    /**
     * Constructor
     */
    NetworkToSurface();

    /**
     * Destructor
     */
    ~NetworkToSurface();

    /**
     * Factory constructor method
     * @return a shared pointer to a new instance
     */
    static std::shared_ptr<NetworkToSurface<DIM> > Create();

    /**
     * Return the tool used to convert the network to an image
     * @return the tool used to convert the network to an image
     */
    std::shared_ptr<NetworkToImage<DIM> > GetNetworkToImageTool();

    /**
     * Return the vessel surface
     * @return a pointer to the vessel surface
     */
    vtkSmartPointer<vtkPolyData> GetSurface();

    /**
     * Whether to do smoothing
     * @param doSmoothing whether to do smoothing
     */
    void SetDoSmoothing(bool doSmoothing);

    /**
     * The number of smoothing iterations
     * @param numIterations the number of smoothing iterations
     */
    void SetNumSmoothingIterations(unsigned numIterations);

    /**
     * The smoothing feature angle
     * @param featureAngle the smoothing feature angle
     */
    void SetSmoothingFeatureAngle(double featureAngle);

    /**
     * The smoothing bandpass frequency
     * @param bandPassFrequency the smoothing bandpass frequency
     */
    void SetBandPassFrequency(double bandPassFrequency);

    /**
     * Set the vessel network
     * @param pNetwork the network to be converted to a surface
     */
    void SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set the re-sampling spline length
     * @param splineResampleSize the spline resampling size
     */
    void SetResamplingSplineSize(units::quantity<unit::length> splineResampleSize);

    /**
     * Do surface remeshing
     * @param doRemeshing do surface remeshing
     */
    void SetDoSurfaceRemeshing(bool doRemeshing);

    /**
     * Remeshing target edge length
     * @param length Remeshing target edge length
     */
    void SetRemeshingTargetEdgeLength(double length);

    /**
     * Do the meshing
     */
    void Update();

};

#endif /* NetworkToSurface_HPP_*/
