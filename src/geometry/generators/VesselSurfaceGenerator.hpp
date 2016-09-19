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

#ifndef VESSELSURFACEGENERATOR_HPP_
#define VESSELSURFACEGENERATOR_HPP_

#include <string>
#include <vtkPlane.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "VesselNetwork.hpp"
#include "Polygon.hpp"

/**
 * Class for generating surface representations of vessel networks. The generated surface is suitable for
 * finite element meshing and can be returned in PLC or VTK formats. Units are not maintained.
 */
template<unsigned DIM>
class VesselSurfaceGenerator
{
    /**
     * The vessel network for which the surface will be generated.
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

    /**
     * A VTK representation of the surface.
     */
    vtkSmartPointer<vtkPolyData> mpSurface;

public:

    /**
     * Constructor
     * @param pVesselNetwork the vessel network to generate the surface on
     */
    VesselSurfaceGenerator(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork);

    /**
     * Destructor
     * @param pVesselNetwork the vessel network to generate the surface on
     */
    ~VesselSurfaceGenerator();

    /**
     * Return the surface in the form of a vector of PLC polygons for each segment
     * @return a vector of PLC polygons for each segment
     */
    std::vector<std::vector<boost::shared_ptr<Polygon> > > GetSurface();

    /**
     * Return the surface in the form of a vector of PLC polygons
     * @return a vector of PLC polygons
     */
    std::vector<boost::shared_ptr<Polygon> > GetSurfacePolygons();

    /**
     * Return the locations of PLC holes in the network. Holes are points that are situated on the
     * 'inside' of vessel segments. They are useful for meshing.
     * @ return the locations of PLC holes in the network
     */
    std::vector<DimensionalChastePoint<DIM> > GetHoles();

    /**
     * Return the surface in the form of VTK polydata
     * @return the surface in the form of VTK polydata
     */
    vtkSmartPointer<vtkPolyData> GetVtkSurface();


private:

    /**
     * Return a vector of locations on a circle centred at x=0, y=0 with specified radius.
     */
    std::vector<c_vector<double, DIM> > MakeCircle(double radius, unsigned numberOfSegments = 16);

    /**
     * Project the input points onto the specified plane
     */
    void ProjectOnPlane(std::vector<c_vector<double, DIM> >& rPoints, c_vector<double, DIM> directionVector, double length,
                        vtkSmartPointer<vtkPlane> plane);

    /**
     * Rotate the input points about the specified axis by the specified angle
     */
    void RotateAboutAxis(std::vector<c_vector<double, DIM> >& rPoints, c_vector<double, DIM> axis, double angle);

    /**
     * Translate the input points along the specified vector
     */
    void Translate(std::vector<c_vector<double, DIM> >& rPoints, c_vector<double, DIM> translationVector);
};

#endif /* VESSELSURFACEGENERATOR_HPP_*/
