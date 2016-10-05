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

#ifndef OWEN11CAUPDATERULE_HPP_
#define OWEN11CAUPDATERULE_HPP_

#include "ChasteSerialization.hpp"
#include <boost/serialization/base_object.hpp>
#include "AbstractCaUpdateRule.hpp"
#include "CaBasedCellPopulation.hpp"
#include "VesselNetwork.hpp"
#include "UnitCollection.hpp"
#include "RegularGrid.hpp"

/**
 * A diffusion update rule based loosely on Owen et al. 2001.
 * Vessels are treated differently to the paper, as they have their own sprouting
 * and migration rules. However they can occupy space.
 *
 * The probability of moving to an adjacent lattice site is
 *
 * D*delta_t*((Nm-N)Nm)/(2*delta_xy)
 *
 * Where D is the mDiffusionParameter.
 * delta_t is the timestep.
 * delta_xy is the separation of the two lattice sites.
 * Nm is the carrying capacity for the cell type
 * N is the number of cells at the lattice site
 *
 * In this manifestation only cancer cells diffuse.
 */
template<unsigned DIM>
class Owen11CaUpdateRule : public AbstractCaUpdateRule<DIM>
{
friend class TestCaUpdateRules;

private:

    /**
     * Diffusion parameter for update rule.
     */
    units::quantity<unit::diffusivity> mDiffusionParameter;

    /**
     * Vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpVesselNetwork;

    /**
     * Regular Grid
     */
    boost::shared_ptr<RegularGrid<DIM> > mpRegularGrid;

    /**
     * Reference length scale
     */
    units::quantity<unit::length> mReferenceLengthScale;

    /**
     * Cancer cell carrying capacity
     */
    unsigned mCancerCellCarryingCapacity;

    friend class boost::serialization::access;
    /**
     * Boost Serialization method for archiving/checkpointing.
     * Archives the object and its member variables.
     *
     * @param archive  The boost archive.
     * @param version  The current version of this class.
     */
    template<class Archive>
    void serialize(Archive & archive, const unsigned int version)
    {
        archive & boost::serialization::base_object<AbstractCaUpdateRule<DIM> >(*this);
        archive & mDiffusionParameter;
        archive & mReferenceLengthScale;
        archive & mCancerCellCarryingCapacity;
    }

public:

    /**
     * Constructor.
     */
    Owen11CaUpdateRule();

    /**
     * Destructor.
     */
    ~Owen11CaUpdateRule();

    /**
     * Calculate the probability of a given move.
     *
     * Uses random diffusion to each neighbouring node, scaled according to distance.
     *
     * @param currentNodeIndex The index of the current node/lattice site
     * @param targetNodeIndex The index of the target node/lattice site
     * @param rCellPopulation The cell population
     * @param dt is the time interval
     * @param deltaX defines the size of the lattice site
     * @param cell a pointer to the cell (needed if more than one cell per lattice site
     * @return The probability of the cell moving from the current node to the target node
     */
    double EvaluateProbability(unsigned currentNodeIndex,
                               unsigned targetNodeIndex,
                               CaBasedCellPopulation<DIM>& rCellPopulation,
                               double dt,
                               double deltaX,
                               CellPtr cell);

    /**
      * @return mDiffusionParameter
      */
    units::quantity<unit::diffusivity> GetDiffusionParameter();

    /**
     * Set the vessel network
     *
     * @param pVesselNetwork The vessel network
     */
    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pVesselNetwork);

    /**
     * Set the regular grid
     *
     * @param pRegularGrid The regular grid
     */
    void SetRegularGrid(boost::shared_ptr<RegularGrid<DIM> > pRegularGrid);

    /**
     * Set the reference length scale
     *
     * @param referenceLengthScale the reference length scale
     */
    void SetReferenceLengthScale(units::quantity<unit::length> referenceLengthScale);

    /**
     * Set mDiffusionParameter.
     *
     * @param diffusionParameter the new value of mDiffusionParameter
     */
    void SetDiffusionParameter(units::quantity<unit::diffusivity> diffusionParameter);

    /**
     * Overridden OutputUpdateRuleParameters() method.
     *
     * @param rParamsFile the file stream to which the parameters are output
     */
    void OutputUpdateRuleParameters(out_stream& rParamsFile);
};

#include "SerializationExportWrapper.hpp"
EXPORT_TEMPLATE_CLASS_SAME_DIMS(Owen11CaUpdateRule)

#endif /*OWEN11CAUPDATERULE_HPP_*/
