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



#ifndef PARTACTORGENERATOR_HPP_
#define PARTACTORGENERATOR_HPP_

#include <vector>
#include "SmartPointers.hpp"
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkLookupTable.h>
#include "Part.hpp"
#include "UnitCollection.hpp"
#include "AbstractActorGenerator.hpp"

/**
 * This class generates Vtk actors for Parts
 */
template<unsigned DIM>
class PartActorGenerator : public AbstractActorGenerator<DIM>
{
    /**
     * The part
     */
    boost::shared_ptr<Part<DIM> > mpPart;

public:

    /**
     * Constructor
     */
    PartActorGenerator();

    /**
     * Destructor
     */
    ~PartActorGenerator();

    /**
     * Add the part actor to the renderer
     * @param pRenderer the current renderer
     */
    void AddActor(vtkSmartPointer<vtkRenderer> pRenderer);

    /**
     * Set the part
     * @param pPart the part to render
     */
    void SetPart(boost::shared_ptr<Part<DIM> > pPart);

};

#endif /* PARTACTORGENERATOR_HPP_*/
