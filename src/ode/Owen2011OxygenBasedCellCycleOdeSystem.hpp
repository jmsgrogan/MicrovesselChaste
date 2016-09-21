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

#ifndef _OWEN2011OXYGENBASEDCELLCYCLEODESYSTEM_HPP_
#define _OWEN2011OXYGENBASEDCELLCYCLEODESYSTEM_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <cmath>
#include "AbstractCellMutationState.hpp"
#include "CancerCellMutationState.hpp"
#include "AbstractOdeSystem.hpp"
#include "UnitCollection.hpp"

/**
 * Represents the Owen (2011) system of ODEs.
 *
 * The variables are
 * 0. phi = Cell-cycle phase
 * 1. p53 = p53 concentration
 * 2. VEGF = VEGF concentration
 * 3. O2 = Oxygen concentration
 */

class Owen2011OxygenBasedCellCycleOdeSystem : public AbstractOdeSystem
{

private:

    /**
     * Partial pressure at half max rate
     */
    units::quantity<unit::pressure> mCphi;

    /**
     * Minimum cycle time
     */
    units::quantity<unit::time> mTmin;

    /**
     * p53 production rate constant
     */
    units::quantity<unit::rate> mk7;

    /**
     * p53 production rate constant
     */
    units::quantity<unit::rate> mk7dash;

    /**
     * Oxygen partial pressure for half-max p53 degradation
     */
    units::quantity<unit::pressure> mCp53;

    /**
     * Parameter with units of min^-1
     */
    units::quantity<unit::rate> mk8;

    /**
     * Parameter with units of min^-1
     */
    units::quantity<unit::rate> mk8doubledash;

    /**
     * Parameter with units of ----
     */
    units::quantity<unit::dimensionless> mJ5;

    /**
     * Parameter with units of min^-1
     */
    units::quantity<unit::rate> mk8dash;

    /**
     * Parameter with units of mmHg
     */
    units::quantity<unit::pressure> mCVEGF;

    /**
     * The oxygen concentration (this affects the ODE system).
     */
    units::quantity<unit::concentration> mOxygenConcentration;

    /**
     * Mutation state.
     */
    boost::shared_ptr<AbstractCellMutationState> pmMutationState;

    /**
     * The reference time scale
     */
    units::quantity<unit::time> mReferenceTimeScale;

    /**
     * The reference concentration scale
     */
    units::quantity<unit::concentration> mReferenceConcentrationScale;

    /**
     * The reference solubility
     */
    units::quantity<unit::solubility> mReferenceSolubility;


    friend class boost::serialization::access;
    /**
     * Serialize the object and its member variables.
     *
     * @param archive the archive
     * @param version the current version of this class
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractOdeSystem>(*this);
    }

public:

    /**
     * Constructor.
     *
     * @param oxygenConcentration the oxygen concentration
     * @param isLabelled whether the cell associated with this cell cycle ODE system is labelled (this affects the ODE system)
     * @param stateVariables optional initial conditions for state variables (only used in archiving)
     */
    Owen2011OxygenBasedCellCycleOdeSystem(units::quantity<unit::concentration> oxygenConcentration,
                                             boost::shared_ptr<AbstractCellMutationState> mutation_state,
                                             std::vector<double> stateVariables=std::vector<double>());

    /**
     * Destructor.
     */
    ~Owen2011OxygenBasedCellCycleOdeSystem();

    /**
     * Compute the RHS of the Owen (2011) system of ODEs.
     *
     * Returns a vector representing the RHS of the ODEs at each time step, y' = [y1' ... yn'].
     * An ODE solver will call this function repeatedly to solve for y = [y1 ... yn].
     *
     * @param time used to evaluate the RHS.
     * @param rY value of the solution vector used to evaluate the RHS.
     * @param rDY filled in with the resulting derivatives (using Owen (2011) system of equations).
     */
    void EvaluateYDerivatives(double time, const std::vector<double>& rY, std::vector<double>& rDY);

    /**
     * Calculate whether the conditions for the cell cycle to finish have been met.
     *
     * @param time at which to calculate whether the stopping event has occurred
     * @param rY value of the solution vector used to evaluate the RHS.
     *
     * @return whether or not stopping conditions have been met
     */
    bool CalculateStoppingEvent(double time, const std::vector<double>& rY);

    /**
     * @return #mOxygenConcentration.
     */
    units::quantity<unit::concentration> GetOxygenConcentration() const;

    /**
     * Get the cell's mutation state.
     */
    boost::shared_ptr<AbstractCellMutationState> GetMutationState() const;

    /**
     * Set the cell's mutation state.
     *
     * @param pMutationState the cell's new mutation state
     */
    void SetMutationState(boost::shared_ptr<AbstractCellMutationState> pMutationState);

};

// Declare identifier for the serializer
#include "SerializationExportWrapper.hpp"
CHASTE_CLASS_EXPORT(Owen2011OxygenBasedCellCycleOdeSystem)

namespace boost
{
namespace serialization
{
/**
 * Serialize information required to construct an Owen2011OxygenBasedCellCycleOdeSystem.
 */
template<class Archive>
inline void save_construct_data(
    Archive & ar, const Owen2011OxygenBasedCellCycleOdeSystem * t, const unsigned int file_version)
{
    // Save data required to construct instance
    const units::quantity<unit::concentration> oxygen_concentration = t->GetOxygenConcentration();
    ar & oxygen_concentration;

    const boost::shared_ptr<AbstractCellMutationState> mutation_state = t->GetMutationState();
    ar & mutation_state;

    const std::vector<double> state_variables = t->rGetConstStateVariables();
    ar & state_variables;
}

/**
 * De-serialize constructor parameters and initialise an Owen2011OxygenBasedCellCycleOdeSystem.
 */
template<class Archive>
inline void load_construct_data(
    Archive & ar, Owen2011OxygenBasedCellCycleOdeSystem * t, const unsigned int file_version)
{
    // Retrieve data from archive required to construct new instance
    units::quantity<unit::concentration> oxygen_concentration;
    ar & oxygen_concentration;

    boost::shared_ptr<AbstractCellMutationState> mutation_state;
    ar & mutation_state;

    std::vector<double> state_variables;
    ar & state_variables;

    // Invoke inplace constructor to initialise instance
    ::new(t)Owen2011OxygenBasedCellCycleOdeSystem(oxygen_concentration, mutation_state, state_variables);
}
}
} // namespace ...

#endif /*_OWEN2011OXYGENBASEDCELLCYCLEODESYSTEM_HPP_*/
