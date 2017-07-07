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

#ifndef NODEFLOWPROPERTIES_HPP_
#define NODEFLOWPROPERTIES_HPP_

//#include <string>
//#include <map>
#include "ChasteSerialization.hpp" // NOLINT
#include "AbstractVesselNetworkComponentFlowProperties.hpp"

/**
 * This is a class for vascular node flow properties.
 *
 * This class stores nodal data for vessel network flow problems. Each node has
 * an instance of the class.
 */
template<unsigned DIM>
class NodeFlowProperties : public std::enable_shared_from_this<NodeFlowProperties<DIM> >, public AbstractVesselNetworkComponentFlowProperties<DIM>
{

private:

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
        #if BOOST_VERSION < 105600
            EXCEPTION("Serialization not supported for Boost < 1.56");
        #else
            ar & boost::serialization::base_object<AbstractVesselNetworkComponentFlowProperties<DIM> >(*this);
            ar & mIsInputNode;
            ar & mIsOutputNode;
            ar & mUseVelocityBoundaryCondition;
        #endif
    }

    /**
     *  Is the node an input node
     */
    bool mIsInputNode;

    /**
     *  Is the node an output node
     */
    bool mIsOutputNode;

    /**
     *  Whether to use the nodal pressure or segment velocity as a boundary condition
     */
    bool mUseVelocityBoundaryCondition;


public:

    /**
     * Constructor
     */
    NodeFlowProperties();

    /**
     * Destructor
     */
    ~NodeFlowProperties();

    /**
     * Return a map of nodal data for use by the vtk writer
     * @return a map of nodal data for use by the vtk writer
     */
    std::map<std::string, double> GetOutputData() const;

    /**
     * Return true if the node is an input node
     * @return true if the node is an input node
     */
    bool IsInputNode() const;

    /**
     * Return true if the node is an output node
     * @return true if the node is an output node
     */
    bool IsOutputNode() const;

    /**
     * Set that the node is an input node
     * @param isInput whether the node is an input
     */
    void SetIsInputNode(bool isInput);

    /**
     *  Set that the node is an output node
     * @param isOutput whether the node is an output
     */
    void SetIsOutputNode(bool isOutput);

    /**
     * Set whether to use inlet velocity boundary conditions for flow calculations
     * @param useVelocity whether to use inlet velocity boundary conditions for flow calculations
     */
    void SetUseVelocityBoundaryCondition(bool useVelocity);

    /**
     * Whether to use inlet velocity boundary conditions for flow calculations
     * @return whether inlet velocity conditions are used
     */
    bool UseVelocityBoundaryCondition();
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS1(NodeFlowProperties, 2)
EXPORT_TEMPLATE_CLASS1(NodeFlowProperties, 3)
#endif /* NODEFLOWPROPERTIES_HPP_ */
