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

#include "UblasIncludes.hpp"
#include "Vertex.hpp"
#include "BaseUnits.hpp"

Vertex::Vertex(double x, double y, double z, units::quantity<unit::length> referenceLength) :
        DimensionalChastePoint<3>(x, y, z, referenceLength),
        mIndex(0)
{
}

Vertex::Vertex(c_vector<double, 3> coords, units::quantity<unit::length> referenceLength) :
        DimensionalChastePoint<3>(coords, referenceLength),
        mIndex(0)
{
}

Vertex::Vertex(double x, double y, double z) :
        DimensionalChastePoint<3>(x, y, z, BaseUnits::Instance()->GetReferenceLengthScale()),
        mIndex(0)
{
}

Vertex::Vertex(c_vector<double, 3> coords) :
        DimensionalChastePoint<3>(coords, BaseUnits::Instance()->GetReferenceLengthScale()),
        mIndex(0)
{
}

boost::shared_ptr<Vertex> Vertex::Create(double x, double y, double z, units::quantity<unit::length> referenceLength)
{
    MAKE_PTR_ARGS(Vertex, pSelf, (x, y, z, referenceLength));
    return pSelf;
}

boost::shared_ptr<Vertex> Vertex::Create(c_vector<double, 3> coords, units::quantity<unit::length> referenceLength)
{
    MAKE_PTR_ARGS(Vertex, pSelf, (coords, referenceLength));
    return pSelf;
}

boost::shared_ptr<Vertex> Vertex::Create(double x, double y, double z)
{
    MAKE_PTR_ARGS(Vertex, pSelf, (x, y, z));
    return pSelf;
}

boost::shared_ptr<Vertex> Vertex::Create(c_vector<double, 3> coords)
{
    MAKE_PTR_ARGS(Vertex, pSelf, (coords));
    return pSelf;
}

Vertex::~Vertex()
{
}

unsigned Vertex::GetIndex()
{
    return mIndex;
}

void Vertex::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    double sin_a = std::sin(angle);
    double cos_a = std::cos(angle);
    c_vector<double, 3> unit_axis = axis / norm_2(axis);

    c_vector<double, 3> old_location = this->mLocation;
    double dot_product = inner_prod(old_location, unit_axis);
    c_vector<double, 3> new_location;
    new_location[0] = (unit_axis[0] * dot_product * (1.0 - cos_a) + old_location[0] * cos_a
                + (-unit_axis[2] * old_location[1] + unit_axis[1] * old_location[2]) * sin_a);
    new_location[1] = (unit_axis[1] * dot_product * (1.0 - cos_a) + old_location[1] * cos_a
                + (unit_axis[2] * old_location[0] - unit_axis[0] * old_location[2]) * sin_a);
    new_location[2] = (unit_axis[2] * dot_product * (1.0 - cos_a) + old_location[2] * cos_a
                + (-unit_axis[1] * old_location[0] + unit_axis[0] * old_location[1]) * sin_a);
    this->mLocation = new_location;
}

void Vertex::SetIndex(unsigned index)
{
    mIndex = index;
}

void Vertex::Translate(c_vector<double, 3> translationVector)
{
    this->mLocation += translationVector;
}
