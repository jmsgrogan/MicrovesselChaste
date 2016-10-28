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

#ifndef MicrovesselVtkScene_HPP_
#define MicrovesselVtkScene_HPP_

#include <vector>
#include "SmartPointers.hpp"
#if VTK_MAJOR_VERSION > 5
    #include <vtkAutoInit.h>
    VTK_MODULE_INIT(vtkRenderingOpenGL);
#endif
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkLookupTable.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include "Part.hpp"
#include "RegularGrid.hpp"
#include "VesselNetwork.hpp"
#include "AbstractCellPopulation.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "UnitCollection.hpp"

/**
 * A simple vtk renderer for simulation outputs
 */
template<unsigned DIM>
class MicrovesselVtkScene
{
    /**
     * The vtk renderer
     */
    vtkSmartPointer<vtkRenderer> mpRenderer;

    /**
     * The vtk render window
     */
    vtkSmartPointer<vtkRenderWindow> mpRenderWindow;

    /**
     * The vtk render window interactor
     */
    vtkSmartPointer<vtkRenderWindowInteractor> mpRenderWindowInteractor;

    /**
     * The part
     */
    boost::shared_ptr<Part<DIM> > mpPart;

    /**
     * The vessel network
     */
    boost::shared_ptr<VesselNetwork<DIM> > mpNetwork;

    /**
     * The mesh
     */
    boost::shared_ptr<DiscreteContinuumMesh<DIM> > mpMesh;

    /**
     * The regular grid
     */
    boost::shared_ptr<RegularGrid<DIM> > mpGrid;

    /**
     * The cell population
     */
    boost::shared_ptr<AbstractCellPopulation<DIM> > mpCellPopulation;

    /**
     * The path for output
     */
    std::string mOutputFilePath;

    /**
     * Scale features using this length. e.g. set to micron if we want
     * to render features in microns
     */
    units::quantity<unit::length> mLengthScale;

    /**
     * The color lookup
     */
    vtkSmartPointer<vtkLookupTable> mpColorLookUpTable;

public:

    /**
     * Constructor
     */
    MicrovesselVtkScene();

    /**
     * Destructor
     */
    ~MicrovesselVtkScene();

    void ResetRenderer();

    void UpdatePartActor();

    void UpdateVesselNetworkActor();

    void UpdateCellPopulationActor();

    void UpdateRegularGridActor();

    void UpdateMeshActor();

    void SetPart(boost::shared_ptr<Part<DIM> > pPart);

    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    void SetCellPopulation(boost::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation);

    void SetRegularGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid);

    void SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh);

    /**
     * Render the scene
     * @param interactive do interactive rendering, otherwise write the scene to file
     */
    void Show(bool interactive = true);

    /**
     * Set the path for output
     * @param rPath the path for output
     */
    void SetOutputFilePath(const std::string& rPath);

};

#endif /* MicrovesselVtkScene_HPP_*/
