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

#ifndef SEGMENTFLOWPROPERTIES_HPP_
#define SEGMENTFLOWPROPERTIES_HPP_

#include <string>
#include <map>
#include <boost/enable_shared_from_this.hpp>
#include <boost/serialization/base_object.hpp>
#include "ChasteSerialization.hpp"
#include "UnitCollection.hpp"
#include "AbstractVesselNetworkComponentFlowProperties.hpp"

/**
 * This is a class for vessel segment flow properties.
 *
 * This class stores segment data for vessel network flow problems. Each segment has
 * an instance of the class.
 */
template<unsigned DIM>
class SegmentFlowProperties : public std::enable_shared_from_this<SegmentFlowProperties<DIM> >, public AbstractVesselNetworkComponentFlowProperties<DIM>
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
            ar & mHaematocrit;
            ar & mFlowRate;
            ar & mImpedance;
            ar & mViscosity;
            ar & mWallShearStress;
            ar & mStimulus;
        #endif
    }

    /**
     * Anti-angiogenic drug concentration in the vessel at this segment
     */
    QConcentration mAntiAngiogenicDrugConcentration;

    /**
     * Haematocrit in the vessel at this segment
     */
    QDimensionless mHaematocrit;

    /**
     * Blood flow rate in the vessel at this segment
     */
    QFlowRate mFlowRate;

    /**
     * Impedance of this vessel segment
     */
    QFlowImpedance mImpedance;

    /**
     * Viscosity of this vessel segment
     */
    QDynamicViscosity mViscosity;

    /**
     * Wall shear stress of this vessel segment
     */
    QPressure mWallShearStress;

    /**
     * Growth stimulus of this vessel segment
     */
    QRate mStimulus;

public:

    /**
     * Constructor
     */
    SegmentFlowProperties();

    /**
     * Destructor
     */
    ~SegmentFlowProperties();

    /**
     * Return the anti-angiogenic drug concentration
     * @return the segment anti-angiogenic drug concentration
     */
    QConcentration GetAntiAngiogenicDrugConcentration() const;

    /**
     * Return the  haematocrit
     * @return the segment haematocrit
     */
    QDimensionless GetHaematocrit() const;

    /**
     * Return the impedance
     * @return the segment impedance
     */
    QFlowImpedance GetImpedance() const;

    /**
     * Return the flow rate
     * @return the segment flow rate
     */
    QFlowRate GetFlowRate() const;

    /**
     * Return the segment viscosity
     * @return the segment viscosity
     */
    QDynamicViscosity GetViscosity() const;

    /**
     * Return the segment wall shear stress
     * @return the segment wall shear stress
     */
    QPressure GetWallShearStress() const;

    /**
     * Return the growth stimulus of this vessel segment
     * @return the segment growth stimulus
     */
    QRate GetGrowthStimulus() const;

    /**
     * Return a map of segment data for use by the vtk writer
     *
     * @return a map of segment data for use by the vtk writer
     */
    std::map<std::string, double> GetOutputData() const;

    /**
     * Set the haematocrit
     *
     * @param haematocrit the haematocrit in the segment
     */
    void SetHaematocrit(QDimensionless haematocrit);

    /**
     * Set the flow rate
     *
     * @param flowRate the flow rate in the segment
     */
    void SetFlowRate(QFlowRate flowRate);

    /**
     * Set the impedance
     *
     * @param impedance the impedance in the segment
     */
    void SetImpedance(QFlowImpedance impedance);

    /**
     * Set the viscosity
     *
     * @param viscosity the viscosity in the segment
     */
    void SetViscosity(QDynamicViscosity viscosity);

    /**
     * Set the wall shear stress of this vessel segment
     *
     * @param wallShear the wall shear stress in the segment
     */
    void SetWallShearStress(QPressure wallShear);

    /**
     * Set the growth stimulus of this vessel segment
     *
     * @param stimulus the growth stimulus in the segment
     */
    void SetGrowthStimulus(QRate stimulus);

};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS1(SegmentFlowProperties, 2)
EXPORT_TEMPLATE_CLASS1(SegmentFlowProperties, 3)

#endif /* SEGMENTFLOWPROPERTIES_HPP_ */
