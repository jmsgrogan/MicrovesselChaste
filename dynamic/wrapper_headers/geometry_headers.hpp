#include "Polygon.hpp"
#include "Facet.hpp"
#include "Part.hpp"
#include "MappableGridGenerator.hpp"
#include "NetworkToSurface.hpp"
#include "VesselSurfaceGenerator.hpp"
#include "VoronoiGenerator.hpp"
#include "BoundaryExtractor.hpp"
#include "SurfaceCleaner.hpp"
#include "GeometryWriter.hpp"

template class Part<3>;
template class Part<2>;
template class Facet<3>;
template class Facet<2>;
template class Polygon<3>;
template class Polygon<2>;
template class NetworkToSurface<3>;
template class VesselSurfaceGenerator<3>;
template class VoronoiGenerator<3>;
template class NetworkToSurface<2>;
template class VesselSurfaceGenerator<2>;
template class VoronoiGenerator<2>;
//typedef std::vector<boost::shared_ptr<Polygon>, std::allocator<boost::shared_ptr<Polygon> > > VecPolygon;
//typedef std::vector<boost::shared_ptr<Vertex>, std::allocator<boost::shared_ptr<Vertex> > > VecVertex;
//typedef std::vector<boost::shared_ptr<Facet>, std::allocator<boost::shared_ptr<Facet> > > VecFacet;
