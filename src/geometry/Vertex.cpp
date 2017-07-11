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

#include "Exception.hpp"
#include "UblasIncludes.hpp"
#include "Vertex.hpp"

template<unsigned DIM>
Vertex<DIM>::Vertex(QLength x, QLength y, QLength z) :
        mIndex(0),
        mAttributes()
{
	c_vector<double, DIM> loc;
    if (DIM > 0)
    {
    	loc[0] = x/1_m;
    }
    if (DIM > 1)
    {
    	loc[1] = y/1_m;
    }
    if (DIM > 2)
    {
    	loc[2] = z/1_m;
    }
    mLocation = VecQLength<DIM>(loc, 1_m);
}

template<unsigned DIM>
Vertex<DIM>::Vertex(const c_vector<double, DIM>& coords, QLength referenceLength) :
        mIndex(0),
        mAttributes()
{
    mLocation = VecQLength<DIM>(coords, referenceLength);
}

template<unsigned DIM>
Vertex<DIM>::Vertex(VecQLength<DIM> loc) :
        mLocation(loc),
        mIndex(0),
        mAttributes()
{

}

template<unsigned DIM>
Vertex<DIM>::Vertex(const double (&rCoords)[3], QLength referenceLength) :
        mIndex(0),
        mAttributes()
{
	c_vector<double, DIM> loc;
    if (DIM > 0)
    {
    	loc[0] = rCoords[0];
    }
    if (DIM > 1)
    {
    	loc[1] = rCoords[1];
    }
    if (DIM > 2)
    {
    	loc[2] = rCoords[2];
    }
    mLocation = VecQLength<DIM>(loc, referenceLength);
}

template<unsigned DIM>
std::shared_ptr<Vertex<DIM> > Vertex<DIM>::Create(QLength x, QLength y, QLength z)
{
    return std::make_shared<Vertex<DIM> >(x, y, z);
}

template<unsigned DIM>
std::shared_ptr<Vertex<DIM> > Vertex<DIM>::Create(const c_vector<double, DIM>& rCoords, QLength referenceLength)
{
    return std::make_shared<Vertex<DIM> >(rCoords, referenceLength);
}

template<unsigned DIM>
std::shared_ptr<Vertex<DIM> > Vertex<DIM>::Create(VecQLength<DIM> loc)
{
    return std::make_shared<Vertex<DIM> >(loc);
}

template<unsigned DIM>
std::shared_ptr<Vertex<DIM> > Vertex<DIM>::Create(const double (&rCoords)[3], QLength referenceLength)
{
    return std::make_shared<Vertex<DIM> >(rCoords, referenceLength);
}

template<unsigned DIM>
std::shared_ptr<Vertex<DIM> > Vertex<DIM>::Create(const Vertex<DIM>& loc)
{
    return std::make_shared<Vertex<DIM> >(loc);
}

template<unsigned DIM>
Vertex<DIM>::~Vertex()
{

}

template<unsigned DIM>
void Vertex<DIM>::AddAttribute(const std::string& rAttribute, double value)
{
    mAttributes[rAttribute] = value;
}

template<unsigned DIM>
std::map<std::string, double> Vertex<DIM>::GetAttributes()
{
    return mAttributes;
}

template<unsigned DIM>
c_vector<double, 3> Vertex<DIM>::Convert3(QLength referenceLength) const
{
    c_vector<double, 3> loc;
    loc[0] = double(mLocation[0]/referenceLength);
    loc[1] = double(mLocation[1]/referenceLength);
    if(DIM==3)
    {
        loc[2] = double(mLocation[2]/referenceLength);
    }
    else
    {
        loc[2] = 0.0;
    }
    return loc;
}

template<unsigned DIM>
c_vector<double, DIM> Vertex<DIM>::Convert(QLength referenceLength) const
{
    return mLocation.Convert(referenceLength);
}

template<unsigned DIM>
void Vertex<DIM>::Convert(double (&rLoc)[3], QLength referenceLength) const
{
    rLoc[0] = double(mLocation[0]/referenceLength);
    rLoc[1] = double(mLocation[1]/referenceLength);
    if(DIM==3)
    {
        rLoc[2] = double(mLocation[2]/referenceLength);
    }
    else
    {
        rLoc[2] = 0.0;
    }
}

template<unsigned DIM>
VecQLength<DIM>& Vertex<DIM>::rGetLocation()
{
    return mLocation;
}

template<unsigned DIM>
const VecQLength<DIM>& Vertex<DIM>::rGetLocation() const
{
    return mLocation;
}

template<unsigned DIM>
QLength Vertex<DIM>::GetDistance(const Vertex<DIM>& rLocation) const
{
    return Qnorm_2(rLocation.rGetLocation() - mLocation);
}

template<unsigned DIM>
QLength Vertex<DIM>::GetNorm2()
{
    return Qnorm_2(mLocation);
}

template<unsigned DIM>
c_vector<double, DIM> Vertex<DIM>::GetUnitVector() const
{
	c_vector<double, DIM> loc = mLocation.Convert(1_m);
    return loc/norm_2(loc);
}

template<unsigned DIM>
Vertex<DIM> Vertex<DIM>::GetMidPoint(const Vertex<DIM>& rLocation) const
{
    return Vertex<DIM>((rLocation.rGetLocation() + mLocation) / 2.0);
}

template<unsigned DIM>
c_vector<double, DIM> Vertex<DIM>::GetUnitTangent(const Vertex<DIM>& rLocation) const
{
    return (rLocation.rGetLocation()- mLocation).Convert(GetDistance(rLocation));
}

template<unsigned DIM>
Vertex<DIM>& Vertex<DIM>::operator/=(double factor)
{
    mLocation = mLocation/factor;
    return *this;
}

template<unsigned DIM>
Vertex<DIM>& Vertex<DIM>::operator*=(double factor)
{
    mLocation = factor * mLocation;
    return *this;
}

template<unsigned DIM>
Vertex<DIM>& Vertex<DIM>::operator+=(const Vertex<DIM>& rLocation)
{
    mLocation = mLocation + rLocation.rGetLocation();
    return *this;
}

template<unsigned DIM>
Vertex<DIM>& Vertex<DIM>::operator-=(const Vertex<DIM>& rLocation)
{
    mLocation = mLocation - rLocation.rGetLocation();
    return *this;
}

template<unsigned DIM>
bool Vertex<DIM>::IsCoincident(const Vertex<DIM>& rLocation) const
{
    bool returned_value = true;
    VecQLength<DIM> comparison_loc = rLocation.rGetLocation();
    for (unsigned dim=0; dim<DIM; dim++)
    {
        if (comparison_loc[dim] != mLocation[dim])
        {
            returned_value = false;
            break;
        }
    }
    return returned_value;
}

template<unsigned DIM>
void Vertex<DIM>::RotateAboutAxis(c_vector<double, 3> axis, double angle)
{
    double sin_a = std::sin(angle);
    double cos_a = std::cos(angle);
    c_vector<double, 3> unit_axis = axis / norm_2(axis);
    if(DIM==2 and unit_axis[2]!= 1.0)
    {
        EXCEPTION("2D rotation is about z axis only");
    }

    c_vector<double, DIM> old_location = this->mLocation.Convert(1_m);
    c_vector<double, DIM> new_location;
    new_location[0] = 0.0;
    new_location[1] = 0.0;

    if(DIM==3)
    {
        double dot_product = inner_prod(old_location, unit_axis);
        new_location[0] = (unit_axis[0] * dot_product * (1.0 - cos_a) + old_location[0] * cos_a
                    + (-unit_axis[2] * old_location[1] + unit_axis[1] * old_location[2]) * sin_a);
        new_location[1] = (unit_axis[1] * dot_product * (1.0 - cos_a) + old_location[1] * cos_a
                    + (unit_axis[2] * old_location[0] - unit_axis[0] * old_location[2]) * sin_a);
        new_location[2] = (unit_axis[2] * dot_product * (1.0 - cos_a) + old_location[2] * cos_a
                    + (-unit_axis[1] * old_location[0] + unit_axis[0] * old_location[1]) * sin_a);
    }
    else
    {
        new_location[0] = old_location[0] * cos_a - old_location[1]*sin_a;
        new_location[1] = old_location[0] * sin_a + old_location[1]*cos_a;
    }

    this->mLocation = VecQLength<DIM>(new_location, 1_m);
}

template<unsigned DIM>
void Vertex<DIM>::Translate(const Vertex<DIM>& rVector)
{
    mLocation = mLocation + rVector.rGetLocation();
}

template<unsigned DIM>
void Vertex<DIM>::TranslateTo(const Vertex<DIM>& rPoint)
{
    mLocation = rPoint.rGetLocation();
}

template<unsigned DIM>
unsigned Vertex<DIM>::GetIndex()
{
    return mIndex;
}

template<unsigned DIM>
void Vertex<DIM>::SetIndex(unsigned index)
{
    mIndex = index;
}

// Explicit instantiation
template class Vertex<1>;
template class Vertex<2>;
template class Vertex<3>;
