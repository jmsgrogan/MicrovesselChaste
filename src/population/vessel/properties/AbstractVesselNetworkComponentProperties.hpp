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

#ifndef ABSTRACTVESSELNETWORKCOMPONENTPROPERTIES_HPP_
#define ABSTRACTVESSELNETWORKCOMPONENTPROPERTIES_HPP_

#include <string>
#include <map>
#include <boost/enable_shared_from_this.hpp>
#include "ChasteSerialization.hpp"
#include "ClassIsAbstract.hpp"
#include "UnitCollection.hpp"

/**
 * This class contains common functionality for property containers for all vessel network components.
 * It provides a common interface that can be used in VTK writers.
 */
template<unsigned DIM>
class AbstractVesselNetworkComponentProperties: public boost::enable_shared_from_this<AbstractVesselNetworkComponentProperties<DIM> >
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

public:

    /**
     * Constructor
     */
    AbstractVesselNetworkComponentProperties();

    /**
     * Destructor
     */
    virtual ~AbstractVesselNetworkComponentProperties();

    /**
     * Return a map of output data for writing to file
     *
     * @return a map of output data for use by writers
     */
    virtual std::map<std::string, double> GetOutputData() const = 0;

};

TEMPLATED_CLASS_IS_ABSTRACT_1_UNSIGNED(AbstractVesselNetworkComponentProperties);

#endif /* ABSTRACTVESSELNETWORKCOMPONENTPROPERTIES_HPP_ */

