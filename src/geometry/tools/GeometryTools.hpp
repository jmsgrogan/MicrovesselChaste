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

#ifndef GEOMETRYTOOLS_HPP_
#define GEOMETRYTOOLS_HPP_

#include <vector>
#include <math.h>
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning for now (gcc4.3)
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include "SmartPointers.hpp"
#include "UblasVectorInclude.hpp"
#include "UnitCollection.hpp"
#include "VectorUnitCollection.hpp"
#include "DimensionalChastePoint.hpp"


/**
 * Return an arbitary (for not random) unit normal for the supplied direction
 * @param direction the direction
 * @return an arbitary (for not random) unit normal
 */
template<unsigned DIM>
c_vector<double, DIM> GetArbitaryUnitNormal(c_vector<double, DIM> direction);

/**
 * Get the distance between two points
 * @param rLocation1 the input point 1
 * @param rLocation2 the input point 2
 * @return the distance between this point and the input point
 */
template<unsigned DIM>
QLength GetDistance(const VecQLength<DIM>& rLocation1, const VecQLength<DIM>& rLocation2);

/**
 * Get the distance between the line defined by the start and end locations and the probe location.
 * @param rStartLocation the start location on the line
 * @param rEndLocation the end location on the line
 * @param rProbeLocation the probe location
 * @return the distance between the line and probe point
 */
template<unsigned DIM>
QLength GetDistanceToLineSegment(const VecQLength<DIM>& rStartLocation, const VecQLength<DIM>& rEndLocation,
        const VecQLength<DIM>& rProbeLocation);

/**
 * Get the dot product of the vectors between each point and the origin
 * @param rLocation1 the input point 1
 * @param rLocation2 the input point 2
 * @return the dot product of the vectors between each point and the origin
 */
template<unsigned DIM>
QArea GetDotProduct(const VecQLength<DIM>& rLocation1, const VecQLength<DIM>& rLocation2);

/**
 * Get the dot product of the vectors between each point and the origin
 * @param rLocation1 the input point 1
 * @param rLocation2 the input point 2
 * @return the dot product of the vectors between each point and the origin
 */
template<unsigned DIM>
QLength GetDotProduct(const VecQLength<DIM>& rLocation1, const c_vector<double, DIM>& rLocation2);

/**
 * Return the projection of a point onto the line defined by the start and end locations
 * @param rStartLocation the start location on the line
 * @param rEndLocation the end location on the line
 * @param rProbeLocation the probe location
 * @param projectToEnds whether to project onto the end points
 * @param checkDimensions check if the dimensions of the input point need to be scaled
 * @return the projection of the probe point onto a line
 */
template<unsigned DIM>
VecQLength<DIM> GetPointProjectionOnLineSegment(const VecQLength<DIM>& rStartLocation,
                                                      const VecQLength<DIM>& rEndLocation,
                                                      const VecQLength<DIM>& rProbeLocation,
                                                      bool projectToEnds = false,
                                                      bool checkDimensions = true);

/**
 * Return a vector of points projected at global normals to the central point a distance probeLength
 * @param rCentrePoint the probe centre
 * @param currentDirection the current direction, nor normalized
 * @param probeLength the probe length
 * @param numDivisions the number of division for gradient calculation
 * @return a vector of points projected at global normals to the central point
 */
template<unsigned DIM>
vtkSmartPointer<vtkPoints> GetProbeLocationsExternalPoint(VecQLength<DIM> rCentrePoint,
        VecQLength<DIM> currentDirection, QLength probeLength,
        unsigned numDivisions=8);

/**
 * Return a vector of points projected at orthogonal directions to the rInitialDirection about the rRotationAxis
 * @param rInitialDirection the initial direction
 * @param rCentralPoint the central point
 * @param rRotationAxis the rotation axis
 * @param probeLength the probe length
 * @param initial rotation angel
 * @return a vector of points projected at global normals to the central point
 */
template<unsigned DIM>
vtkSmartPointer<vtkPoints> GetProbeLocationsInternalPoint(VecQLength<DIM> rInitialDirection,
        VecQLength<DIM> rCentralPoint, VecQLength<DIM> rRotationAxis, QLength probeLength,
                                                                         QAngle angle);

/**
 * Is the point inside the cone defined by apex, aperture and  base centre
 * @param rPoint the point
 * @param rApex the cone apex
 * @param rBase the cone base
 * @param aperature the aperature
 * @return is the point inside the cone
 */
template<unsigned DIM>
bool IsPointInCone(const VecQLength<DIM>& rPoint,
                   const VecQLength<DIM>& rApex,
                   const VecQLength<DIM>& rBase, double aperture);

/**
 * Is the point inside the cube box defined by a centre location and box side length
 * @param rPoint the point
 * @param rLocation the centre of the box
 * @param spacing the box spacing
 * @return is the point inside the box
 */
template<unsigned DIM>
bool IsPointInBox(const VecQLength<DIM>& rPoint,
                  const VecQLength<DIM>& rLocation, QLength spacing);

/**
 * Is the point inside the cube box defined by a centre location and bounds
 * @param rPoint the point
 * @param rBoundingBox the dimensionless bounds
 * @param lengthScale the reference length scale
 * @return is the point inside the box
 */
template<unsigned DIM>
bool IsPointInBox(const VecQLength<DIM>& rPoint,
        const c_vector<double, 6>& rBoundingBox, QLength lengthScale);

/**
 * Is the point inside the tetrahedron given by the vector of vertex locations
 * @param rPoint the point
 * @param locations the tetrahedron vertices
 * @return is the point inside the tetrahedron
 */
template<unsigned DIM>
bool IsPointInTetra(const VecQLength<DIM>& rPoint, const std::vector<VecQLength<DIM> >& locations);

/**
 * Return the length of the line given by a start point and end point in the box given by a centre location and side length
 * @param rStartPoint the line start
 * @param rEndPoint the line end
 * @param spacing the box spacing
 * @return the length of the line in the box
 */
template<unsigned DIM>
QLength LengthOfLineInBox(const VecQLength<DIM>& rStartPoint,
                         const VecQLength<DIM>& rEndPoint,
                         const VecQLength<DIM>& rBoxCentre, QLength spacing);

/**
 * Return the length of the line given by a start point and end point
 * @param rStartPoint the line start
 * @param rEndPoint the line end
 * @param bounding_box the dimensionless box dimensions
 * @param reference_length length scale for the box dimensions
 * @return the length of the line in the box
 */
template<unsigned DIM>
QLength LengthOfLineInBox(const VecQLength<DIM>& rStartPoint,
                         const VecQLength<DIM>& rEndPoint,
                         const c_vector<double, 6>& rBoundingBox,
                         QLength lengthScale);

/**
 * Return the length of the line given by a start point and end point in the tetrahedron given by vertex locations
 * @param rStartPoint the line start
 * @param rEndPoint the line end
 * @param locations the tetrahedron vertices
 * @return the length of the line in the tetrahedron
 */
template<unsigned DIM>
QLength LengthOfLineInTetra(const VecQLength<DIM>& rStartPoint,
                                                  const VecQLength<DIM>& rEndPoint,
                           const std::vector<VecQLength<DIM> >& locations);

/**
 * Return a point offset a prescribed distance along the unit vector of the input vector. Used by sprouting rules
 * @param rVector the direction
 * @param offset the offset along the direction
 * @return a point offset a prescribed distance along the direction
 */
template<unsigned DIM>
VecQLength<DIM> OffsetAlongVector(const VecQLength<DIM>& rVector, QLength offset);

/**
 * Return a point offset a prescribed distance along the unit vector of the input vector. Used by sprouting rules
 * @param rVector the direction
 * @param offset the offset along the direction
 * @param referenceLength the reference length for the vector
 * @return a point offset a prescribed distance along the direction
 */
template<unsigned DIM>
VecQLength<DIM> OffsetAlongVector(const c_vector<double, DIM>& rVector, QLength offset,
                                              QLength referenceLength);

/**
 * Rotate the supplied vector about the axis by the specified angle.
 * @param rDirection the current direction
 * @param rAxis the rotation axis
 * @param angle the rotation angle (radians)
 * @return the rotate vector as a point
 */
template<unsigned DIM>
VecQLength<DIM> RotateAboutAxis(const VecQLength<DIM>& rDirection, const c_vector<double, 3>& rAxis, QAngle angle);

/**
 * Rotate the supplied vector about the axis by the specified angle.
 * @param rDirection the current direction
 * @param rAxis the rotation axis
 * @param angle the rotation angle (radians)
 * @return the rotate vector as a point
 */
template<unsigned DIM>
c_vector<double, DIM> RotateAboutAxis(const c_vector<double, DIM>& rDirection,
                                      const c_vector<double, 3>& rAxis, QAngle angle);

#include "GeometryToolsImpl.hpp"
#endif /*GEOMETRYTOOLS_HPP_*/
