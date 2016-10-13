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

#ifndef NETWORKTOIMAGE_HPP_
#define NETWORKTOIMAGE_HPP_

#include "SmartPointers.hpp"
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkImageData.h>
#include <vtkSmartPointer.h>
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"

/**
* Convert a vessel network to vtk image data. This can be used for visualization
* or as input to centreline/mesh tools.
*/
template<unsigned DIM>
class NetworkToImage
{
    /**
     *  The image
     */
    vtkSmartPointer<vtkImageData> mpImage;

    /**
     *  The vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     *  The grid spacing
     */
    units::quantity<unit::length> mGridSpacing;

    /**
     *  The padding factors in each direction
     */
    c_vector<double, DIM> mPaddingFactors;

    /**
     *  The dimension of the output image
     */
    unsigned mImageDimension;


public:

    /**
     * Constructor
     */
    NetworkToImage();

    /**
     * Destructor
     */
    ~NetworkToImage();

    /**
     * Factory constructor method
     * @return a pointer to the class
     */
    static boost::shared_ptr<NetworkToImage<DIM> > Create();

    /**
     * Get the image
     * @return a pointer to the image representation of the network
     */
    vtkSmartPointer<vtkImageData> GetOutput();

    /**
     * Set the vessel network
     * @param pNetwork the network
     */
    void SetNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
     * Set the pixel spacing for the image
     * @param spacing the pixel spacing
     */
    void SetGridSpacing(units::quantity<unit::length> spacing);

    /**
     * Set the padding factors for the image. This is multiplied by the network extents
     * to add padding. Eg. final_extent = original_extent * (1+padding factor)
     * @param paddingX the X padding factor
     * @param paddingY the Y padding factor
     * @param paddingZ the Z padding factor
     */
    void SetPaddingFactors(double paddingX, double paddingY, double paddingZ);

    /**
     * Set the image dimension. Although the class is templated over spatial dimension, it is often of interest to make
     * 2D images. The image dimension can be set here.
     * @param dimension the image dimension
     */
    void SetImageDimension(unsigned dimension);

    /**
     * Do the conversion
     */
    void Update();
};

#endif /*NETWORKTOIMAGE_HPP_*/
