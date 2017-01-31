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

#ifndef GEOMETRY_HEADERS_HPP
#define GEOMETRY_HEADERS_HPP

#include "Polygon.hpp"
#include "Facet.hpp"
#include "Part.hpp"
#include "MappableGridGenerator.hpp"
#include "NetworkToSurface.hpp"
#include "VesselSurfaceGenerator.hpp"
#include "BoundaryExtractor.hpp"
#include "SurfaceCleaner.hpp"
#include "GeometryWriter.hpp"
#include "DimensionalChastePoint.hpp"

//// Adding typedef's in this namespace will give nicer class names
namespace pyplusplus{
namespace aliases{
typedef Part<3> Part3;
typedef Part<2> Part2;
typedef Facet<3> Facet3;
typedef Facet<2> Facet2;
typedef Polygon<3> Polygon3;
typedef Polygon<2> Polygon2;
typedef NetworkToSurface<3> NetworkToSurface3;
typedef VesselSurfaceGenerator<3> VesselSurfaceGenerator3;
typedef NetworkToSurface<2> NetworkToSurface2;
typedef VesselSurfaceGenerator<2> VesselSurfaceGenerator2;
typedef std::vector<std::vector<boost::shared_ptr<Polygon<2> >, std::allocator<boost::shared_ptr<Polygon<2> > > > > VecVecPolygonPtr2;
typedef std::vector<std::vector<boost::shared_ptr<Polygon<3> >, std::allocator<boost::shared_ptr<Polygon<3> > > > > VecVecPolygonPtr3;
typedef std::vector<boost::shared_ptr<Polygon<2> >, std::allocator<boost::shared_ptr<Polygon<2> > > > VecPolygonPtr2;
typedef std::vector<boost::shared_ptr<Polygon<3> >, std::allocator<boost::shared_ptr<Polygon<3> > > > VecPolygonPtr3;
typedef std::vector<boost::shared_ptr<Facet<2> >, std::allocator<boost::shared_ptr<Facet<2> > > > VecFacetPtr2;
typedef std::vector<boost::shared_ptr<Facet<3> >, std::allocator<boost::shared_ptr<Facet<3> > > > VecFacetPtr3;
typedef std::vector<boost::shared_ptr<DimensionalChastePoint<2> >, std::allocator<boost::shared_ptr<DimensionalChastePoint<2> > > > VecDimensionalChastePointPtr2;
typedef std::vector<boost::shared_ptr<DimensionalChastePoint<3> >, std::allocator<boost::shared_ptr<DimensionalChastePoint<3> > > > VecDimensionalChastePointPtr3;
typedef std::vector<units::quantity<unit::length> > VecLengthQuantity;

// These are defined in the mesh module. They are included here to avoid duplicate registration warnings.
typedef std::vector<DimensionalChastePoint<2>, std::allocator<DimensionalChastePoint<2> > > VecDimensionalChastePoint2_GeometryModule;
typedef std::vector<DimensionalChastePoint<3>, std::allocator<DimensionalChastePoint<3> > > VecDimensionalChastePoint3_GeometryModule;

// These are defined in the PyChaste preload module. They are included here to avoid duplicate registration warnings.
typedef std::vector<unsigned int> VecUnsignedInt_GeometryModule;
typedef std::vector<bool> VecBool_GeometryModule;
typedef std::vector< std::pair<unsigned int, unsigned int> > VecPairUnsignedIntUnsignedInt_GeometryModule;

}
}//pyplusplus::aliases

// Instantiate template classes
template class Part<3>;
template class Part<2>;
template class Facet<3>;
template class Facet<2>;
template class Polygon<3>;
template class Polygon<2>;
template class NetworkToSurface<3>;
template class VesselSurfaceGenerator<3>;
template class NetworkToSurface<2>;
template class VesselSurfaceGenerator<2>;

#endif
