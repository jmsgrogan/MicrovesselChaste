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



#ifndef DENSITYMAP_HPP_
#define DENSITYMAP_HPP_

#include "SmartPointers.hpp"
#include "AbstractCellMutationState.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"

/**
 * Calculate the density of vessel network features (nodes, branches, segments) or cells on a
 * grid using a range of measures.
 */
template<unsigned DIM>
class DensityMap : public AbstractRegularGridDiscreteContinuumSolver<DIM>
{
    /**
     * Dimensionless vessel surface area density
     */
    std::vector<double> mVesselSurfaceAreaDensity;

    /**
     * Dimensionless vessel line density
     */
    std::vector<double> mVesselLineDensity;

    /**
     * Dimensionless perfused vessel surface area density
     */
    std::vector<double> mPerfusedVesselSurfaceAreaDensity;

    /**
     * Dimensionless perfused vessel line area density
     */
    std::vector<double> mPerfusedVesselLineDensity;

    /**
     * Dimensionless vessel tip density
     */
    std::vector<double> mVesselTipDensity;

    /**
     * Dimensionless vessel branch density
     */
    std::vector<double> mVesselBranchDensity;

    /**
     * Dimensionless vessel quantity density
     */
    std::vector<double> mVesselQuantityDensity;

    /**
     * Dimensionless cell density
     */
    std::vector<double> mDimensionlessCellDensity;

    /**
     * Dimensionless cell density by mutation type
     */
    std::vector<double> mDimensionlessCellDensityByMutationType;

public:

    /**
     * Constructor
     */
    DensityMap();

    /**
     * Factory constructor method
     * @return a shared pointer to a new solver
     */
    static boost::shared_ptr<DensityMap<DIM> > Create();

    /**
     * Destructor
     */
    ~DensityMap();

    /**
     * Get the vessel surface area density
     * @param update update the stored quantity
     * @return the vessel surface area density
     */
    std::vector<double> GetVesselSurfaceAreaDensity(bool update=true);

    /**
     * Get the vessel line density
     * @param update update the stored quantity
     * @return the vessel line density
     */
    std::vector<double> GetVesselLineDensity(bool update=true);

    /**
     * Get the perfused vessel surface area density
     * @param update update the stored quantity
     * @return the perfused vessel surface area density
     */
    std::vector<double> GetPerfusedVesselSurfaceAreaDensity(bool update=true);

    /**
     * Get the perfused vessel line density
     * @param update update the stored quantity
     * @return the perfused vessel line  density
     */
    std::vector<double> GetPerfusedVesselLineDensity(bool update=true);

    /**
     * Get the vessel tip density
     * @param update update the stored quantity
     * @return the vessel tip density
     */
    std::vector<double> GetVesselTipDensity(bool update=true);

    /**
     * Get the vessel branch density
     * @param update update the stored quantity
     * @return the vessel branch density
     */
    std::vector<double> GetVesselBranchDensity(bool update=true);

    /**
     * Get the vessel quantity density
     * @param rQuantity the quantity to update
     * @param update update the stored quantity
     * @return the vessel quantity density
     */
    std::vector<double> GetVesselQuantityDensity(const std::string& rQuantity, bool update=true);

    /**
     * Get the cell density
     * @param update update the stored quantity
     * @return the cell density
     */
    std::vector<double> GetCellDensity(bool update=true);

    /**
     * Get the cell by mutation state
     * @param pMutationState the cell mutation state to check
     * @param update update the stored quantity
     * @return the cell density by mutation state
     */
    std::vector<double> GetCellDensity(boost::shared_ptr<AbstractCellMutationState> pMutationState, bool update=true);

    /**
     * Null solve method, need to override abstract base class.
     */
    void Solve();

};

#endif /* DENSITYMAP_HPP_ */
