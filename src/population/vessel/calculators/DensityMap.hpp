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

#ifndef DENSITYMAP_HPP_
#define DENSITYMAP_HPP_

#include "SmartPointers.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"

/**
 * Calculate the density of vessel network features (nodes, branches, segments) or cells on a
 * regular grid using a range of methods.
 *
 * In all cases grid points are assigned a box, centred at the point,
 * with side length equal to the grid spacing. Boundary grid points have smaller boxes,
 * cut off where the domain naturally ends. i.e. they get half a box on edges in 2D, and quarter of
 * a box on corners in 2D.
 *
 * Vessel Methods: Point Based
 * If a vessel node is at a point, the quantity density is increased by half the grid spacing per box volume.
 * Optionally, 3D densities can be collapsed to 2D by checking if any vessel nodes in the out of plane direction
 * at a given grid point, similar to how a photo of a 3D feature would appear.
 *
 * Vessel Methods: Line Based
 * Vessel line densities are calculated as the length of all vessels inside the box, divided by box volume.
 *
 * Cell Methods: Point Based
 * If a cell is a a box increase the number of cells per box
 *
 */
template<unsigned DIM>
class DensityMap : public AbstractRegularGridDiscreteContinuumSolver<DIM>
{
    /**
     * Use the surface based vessel density
     */
    bool mUseSurfaceBasedVesselDensity;

    /**
     * Use the line based vessel density
     */
    bool mUseLineBasedVesselDensity;

    /**
     * Use the point based vessel density
     */
    bool mUsePointBasedVesselDensity;

    /**
     * Use the cell density
     */
    bool mUseCellDensity;

    /**
     * Store the 2D projection of the density
     */
    std::vector<double> m2dProjectedDensity;

    /**
     * Store the 1D projection of the density
     */
    std::vector<double> m1dProjectedDensity;

    /**
     * Calculate the branch density
     */
    bool mUseBranchDensity;

    /**
     * Calculate the tip density
     */
    bool mUseTipDensity;

    /**
     * Calculate the perfused density
     */
    bool mUsePerfusedDensity;

    /**
     * Calculate the regressing
     */
    bool mRegressingDensity;

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
     * @return the projection of the density onto 2D
     */
    std::vector<double> Get2dProjectedDensity();

    /**
     * @return the projection of the density onto 1D
     */
    std::vector<double> Get1dProjectedDensity();

    /**
     * Set whether to use the line based vessel density
     * @param useLineBased whether to use the line based vessel density
     */
    void SetUseLineBasedVesselDensity(bool useLineBased);

    /**
     * Set whether to use the surface based vessel density
     * @param useSurfaceBased whether to use the surface based vessel density
     */
    void SetUseSurfaceBasedVesselDensity(bool useSurfaceBased);

    /**
     * Set whether to use the point based vessel density
     * @param usePointBased whether to use the point based vessel density
     */
    void SetUsePointBasedVesselDensity(bool usePointBased);

    /**
     * Set whether to use the cell density
     * @param useCellBased whether to use the cell density
     */
    void SetUseCellDensity(bool useCellBased);

    /**
     * Set whether to use the branch density
     * @param useBranchBased whether to use the branch density
     */
    void SetUseBranchDensity(bool useBranchBased);

    /**
     * Set whether to use the tip density
     * @param useTipBased whether to use the tip density
     */
    void SetUseTipDensity(bool useTipBased);

    /**
     * Set whether to use the perfused density
     * @param usePerfusedBased whether to use the perfused density
     */
    void SetUsePerfusedDensity(bool usePerfusedBased);

    /**
     * Set whether to use the regressing density
     * @param useRegressingBased whether to use the regressing density
     */
    void SetUseRegressingDensity(bool useRegressingBased);

    /**
     * Calculate the map
     */
    void Solve();

};

#endif /* DENSITYMAP_HPP_ */
