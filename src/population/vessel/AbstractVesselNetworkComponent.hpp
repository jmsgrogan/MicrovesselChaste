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

#ifndef ABSTRACTVESSELNETWORKCOMPONENT_HPP_
#define ABSTRACTVESSELNETWORKCOMPONENT_HPP_

#include <vector>
#include <string>
#include <map>
#include <boost/enable_shared_from_this.hpp>
#include "ChasteSerialization.hpp"
#include "ClassIsAbstract.hpp"
#include "UblasVectorInclude.hpp"
#include "UnitCollection.hpp"
#include "SmartPointers.hpp"
#include "Exception.hpp"
#include "AbstractVesselNetworkComponentProperties.hpp"

/**
 * This class contains functionality common to all components of a vessel network.
 */
template<unsigned DIM>
class AbstractVesselNetworkComponent
{
    /**
     * Archiving
     */
    friend class boost::serialization::access;

    /**
     * Do the serialize
     * @param ar the archive
     * @param version the archive version number
     */
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {

    }

protected:

    /**
     * Container for generic component data.
     */
    std::map<std::string, double> mOutputData;

    /**
     * Id tag, useful for post-processing
     */
    unsigned mId;

    /**
     * The component radius
     */
    QLength mRadius;

public:

    /**
     * Constructor.
     */
    AbstractVesselNetworkComponent();

    /**
     * Destructor
     */
    virtual ~AbstractVesselNetworkComponent();

    /**
     * Return the component Id
     *
     * @return the component id
     */
    virtual unsigned GetId() const;

    /**
     * Return the output data for the given key. This method is relatively slow compared to GetOutputData as the
     * data map is reconstructed each time. It is used by the Python framework.
     *
     * @param rKey the key to be queried
     * @return the component data for the input key
     */
    virtual double GetOutputDataValue(const std::string& rKey);

    /**
     * Return a map of output data for writers
     * @return a map of component data for use by the vtk writer
     */
    virtual std::map<std::string, double> GetOutputData() = 0;

    /**
     * Return the keys of the output data map
     * @return a map of component data for use by the vtk writer
     */
    virtual std::vector<std::string> GetOutputDataKeys();

    /**
     * Return the radius of the component
     *
     * @return the radius of the component
     */
    virtual QLength GetRadius() const;

    /**
     * Assign the Id
     * @param id the id for the component
     */
    virtual void SetId(unsigned id);

    /**
     * Add output data to the component using the identifying key
     * @param rKey the key for the data being assigned to the node
     * @param value the value to be stored
     */
    virtual void SetOutputData(const std::string& rKey, double value);

    /**
     * Set the component radius
     * @param radius the component radius
     */
    virtual void SetRadius(QLength radius);

};

TEMPLATED_CLASS_IS_ABSTRACT_1_UNSIGNED(AbstractVesselNetworkComponent);

#endif /* ABSTRACTVESSELNETWORKCOMPONENT_HPP_ */
