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

#ifndef ARTIFICIALIMAGEGENERATOR_HPP_
#define ARTIFICIALIMAGEGENERATOR_HPP_

#include "SmartPointers.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"

/**
 * Calculate the density of vessel network features (nodes, branches, segments) or cells on a
 * grid using a range of methods.
 *
 */
template<unsigned DIM>
class ArtificialImageGenerator : public AbstractRegularGridDiscreteContinuumSolver<DIM>
{
//    /**
//     * Dimensionless vessel surface area density
//     */
//    std::vector<double> mVesselSurfaceAreaDensity;
//
//    /**
//     * Dimensionless vessel line area density
//     */
//    std::vector<double> mVesselLineAreaDensity;
//
//    /**
//     * Dimensionless perfused vessel surface area density
//     */
//    std::vector<double> mPerfusedVesselSurfaceAreaDensity;
//
//    /**
//     * Dimensionless perfused vessel line area density
//     */
//    std::vector<double> mPerfusedVesselLineAreaDensity;
//
//    /**
//     * Dimensionless vessel tip density
//     */
//    std::vector<double> mVesselTipDensity;
//
//    /**
//     * Dimensionless vessel branch density
//     */
//    std::vector<double> mVesselBranchDensity;
//
//    /**
//     * Dimensionless vessel quantity density
//     */
//    std::vector<double> mVesselQuantityDensity;
//
//    /**
//     * Dimensionless cell density
//     */
//    std::vector<double> mDimensionlessCellDensity;
//
//    /**
//     * Dimensionless cell density by mutation type
//     */
//    std::vector<double> mDimensionlessCellDensityByMutationType;

public:

    /**
     * Constructor
     */
    ArtificialImageGenerator();

    /**
     * Factory constructor method
     * @return a shared pointer to a new solver
     */
    static boost::shared_ptr<ArtificialImageGenerator<DIM> > Create();

    /**
     * Destructor
     */
    ~ArtificialImageGenerator();

//    /**
//     * @return the projection of the density onto 2D
//     */
//    std::vector<double> Get2dProjectedDensity();
//
//    /**
//     * @return the projection of the density onto 1D
//     */
//    std::vector<double> Get1dProjectedDensity();
//
//    /**
//     * Set whether to use the line based vessel density
//     * @param useLineBased whether to use the line based vessel density
//     */
//    void SetUseLineBasedVesselDensity(bool useLineBased);
//
//    /**
//     * Set whether to use the surface based vessel density
//     * @param useSurfaceBased whether to use the surface based vessel density
//     */
//    void SetUseSurfaceBasedVesselDensity(bool useSurfaceBased);
//
//    /**
//     * Set whether to use the point based vessel density
//     * @param usePointBased whether to use the point based vessel density
//     */
//    void SetUsePointBasedVesselDensity(bool usePointBased);
//
//    /**
//     * Set whether to use the cell density
//     * @param useCellBased whether to use the cell density
//     */
//    void SetUseCellDensity(bool useCellBased);
//
//    /**
//     * Set whether to use the branch density
//     * @param useBranchBased whether to use the branch density
//     */
//    void SetUseBranchDensity(bool useBranchBased);
//
//    /**
//     * Set whether to use the tip density
//     * @param useTipBased whether to use the tip density
//     */
//    void SetUseTipDensity(bool useTipBased);
//
//    /**
//     * Set whether to use the perfused density
//     * @param usePerfusedBased whether to use the perfused density
//     */
//    void SetUsePerfusedDensity(bool usePerfusedBased);
//
//    /**
//     * Set whether to use the regressing density
//     * @param useRegressingBased whether to use the regressing density
//     */
//    void SetUseRegressingDensity(bool useRegressingBased);

    /**
     * Calculate the map
     */
    void Solve()
    {

    }

};

#endif /* ARTIFICIALIMAGEGENERATOR_HPP_ */
