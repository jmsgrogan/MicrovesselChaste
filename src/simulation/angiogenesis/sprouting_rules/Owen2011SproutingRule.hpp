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

#ifndef OWEN2011SPROUTINGRULE_HPP_
#define OWEN2011SPROUTINGRULE_HPP_

#include <vector>
#include <string>
#include "VesselNode.hpp"
#include "SmartPointers.hpp"
#include "LatticeBasedSproutingRule.hpp"
#include "RegularGrid.hpp"
#include "AbstractRegularGridDiscreteContinuumSolver.hpp"

/**
 * A sprouting rule based on the paper of Owen et al. 2011
 */
template<unsigned DIM>
class Owen2011SproutingRule : public LatticeBasedSproutingRule<DIM>
{

protected:

    /**
     * The half maximum vegf
     */
    units::quantity<unit::concentration> mHalfMaxVegf;

    /**
     * The vegf field sampled at the vessel lattice sites
     */
    std::vector<units::quantity<unit::concentration> > mVegfField;

public:

    /**
     * Constructor.
     */
    Owen2011SproutingRule();

    /**
     * Destructor.
     */
    virtual ~Owen2011SproutingRule();

    /**
     * Construct a new instance of the class and return a shared pointer to it.
     * @return a pointer to a new instance of the class
     */
    static boost::shared_ptr<Owen2011SproutingRule<DIM> > Create();

    /**
     * Overwritten method to return nodes which may sprout
     * @param rNodes nodes to check for sprouting
     * @return a vector of nodes which may sprout
     */
    virtual std::vector<boost::shared_ptr<VesselNode<DIM> > > GetSprouts(const std::vector<boost::shared_ptr<VesselNode<DIM> > >& rNodes);

    /**
     * Set the half max vegf
     * @param halfMaxVegf the half max vegf
     */
    void SetHalfMaxVegf(units::quantity<unit::concentration> halfMaxVegf);
};

#endif /* OWEN2011SPROUTINGRULE_HPP_ */
