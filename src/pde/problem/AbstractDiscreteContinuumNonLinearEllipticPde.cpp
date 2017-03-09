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
 * Redistributions in binary form must reproduce the abovea copyright notice,
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

#include <algorithm>
#include "AbstractDiscreteContinuumNonLinearEllipticPde.hpp"

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumNonLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::AbstractDiscreteContinuumNonLinearEllipticPde() :
            AbstractNonlinearEllipticPde<SPACE_DIM>(),
            AbstractDiscreteContinuumPde<ELEMENT_DIM, ELEMENT_DIM>()
{
    mDiffusionTensor *= this->mDiffusivity.value();
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
AbstractDiscreteContinuumNonLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::~AbstractDiscreteContinuumNonLinearEllipticPde()
{

}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::concentration_flow_rate> AbstractDiscreteContinuumNonLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeConstantInUSourceTerm(unsigned gridIndex)
{
    if(this->mDiscreteConstantSourceStrengths.size()>0)
    {
        if(gridIndex >= this->mDiscreteConstantSourceStrengths.size())
        {
            EXCEPTION("Requested out of bound grid index in discrete sources. Maybe you forgot to update the source strengths.");
        }
        return this->mConstantInUTerm + this->mDiscreteConstantSourceStrengths[gridIndex];
    }
    else
    {
        return this->mConstantInUTerm;
    }
}

template<unsigned ELEMENT_DIM, unsigned SPACE_DIM>
units::quantity<unit::rate> AbstractDiscreteContinuumNonLinearEllipticPde<ELEMENT_DIM, SPACE_DIM>::ComputeLinearInUCoeffInSourceTerm(unsigned gridIndex)
{
    if(this->mDiscreteLinearSourceStrengths.size()>0)
    {
        if(gridIndex >= this->mDiscreteLinearSourceStrengths.size())
        {
            EXCEPTION("Requested out of bound grid index in discrete sources. Maybe you forgot to update the source strengths.");
        }
        return this->mLinearInUTerm + this->mDiscreteLinearSourceStrengths[gridIndex];
    }
    else
    {
        return this->mLinearInUTerm;
    }
}

// Explicit instantiation
template class AbstractDiscreteContinuumNonLinearEllipticPde<2>;
template class AbstractDiscreteContinuumNonLinearEllipticPde<3>;
