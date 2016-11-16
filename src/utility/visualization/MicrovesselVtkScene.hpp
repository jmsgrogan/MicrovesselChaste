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

#ifndef MICROVESSELVTKSCENE_HPP_
#define MICROVESSELVTKSCENE_HPP_

#include <vector>
#include "SmartPointers.hpp"
#include <vtkVersion.h>
#if VTK_MAJOR_VERSION > 5
    #include <vtkAutoInit.h>
    VTK_MODULE_INIT(vtkRenderingOpenGL);
    VTK_MODULE_INIT(vtkRenderingFreeType);
    #include <vtkOggTheoraWriter.h>
#endif
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkLookupTable.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkWindowToImageFilter.h>
#include "Part.hpp"
#include "PartActorGenerator.hpp"
#include "RegularGrid.hpp"
#include "RegularGridActorGenerator.hpp"
#include "VesselNetwork.hpp"
#include "VesselNetworkActorGenerator.hpp"
#include "AbstractCellPopulation.hpp"
#include "CellPopulationActorGenerator.hpp"
#include "DiscreteContinuumMesh.hpp"
#include "DiscreteContinuumMeshActorGenerator.hpp"
#include "UnitCollection.hpp"
#include "MeshBasedCellPopulation.hpp"

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
     * The path for output
     */
    std::string mOutputFilePath;

    /**
     * The color lookup
     */
    vtkSmartPointer<vtkLookupTable> mpColorLookUpTable;

    #if VTK_MAJOR_VERSION > 5
    /**
     * The animation writer
     */
    vtkSmartPointer<vtkOggTheoraWriter> mAnimationWriter;
    #endif

    /**
     * The image to window filter
     */
    vtkSmartPointer<vtkWindowToImageFilter> mWindowToImageFilter;

    /**
     * Is the rendering interactive
     */
    bool mIsInteractive;

    /**
     * Save as an animation
     */
    bool mSaveAsAnimation;

    /**
     * Save as an image
     */
    bool mSaveAsImages;

    /**
     * Has the renderer started
     */
    bool mHasStarted;

    /**
     * Add annotation
     */
    bool mAddAnnotations;

    /**
     * How often to update the renderer during a simulation
     */
    unsigned mOutputFrequency;

    bool mIncludeAxes;

    /**
     * The cell population
     */
    boost::shared_ptr<CellPopulationActorGenerator<DIM> > mpCellPopulationGenerator;

    /**
     * The part generator
     */
    boost::shared_ptr<PartActorGenerator<DIM> > mpPartGenerator;

    /**
     * The vessel network
     */
    boost::shared_ptr<VesselNetworkActorGenerator<DIM> > mpNetworkGenerator;

    /**
     * The mesh
     */
    boost::shared_ptr<DiscreteContinuumMeshActorGenerator<DIM> > mpDiscreteContinuumMeshGenerator;

    /**
     * The regular grid actor generator
     */
    boost::shared_ptr<RegularGridActorGenerator<DIM> > mpGridGenerator;

    /**
     * Scale features using this length. e.g. set to micron if we want
     * to render features in microns
     */
    units::quantity<unit::length> mLengthScale;

public:

    /**
     * Constructor
     */
    MicrovesselVtkScene();

    /**
     * Destructor
     */
    ~MicrovesselVtkScene();

    /**
     * Shut down the scene and close the animation
     */
    void End();

    boost::shared_ptr<PartActorGenerator<DIM> > GetPartActorGenerator();

    boost::shared_ptr<DiscreteContinuumMeshActorGenerator<DIM> > GetDiscreteContinuumMeshActorGenerator();

    boost::shared_ptr<RegularGridActorGenerator<DIM> > GetRegularGridActorGenerator();

    boost::shared_ptr<VesselNetworkActorGenerator<DIM> > GetVesselNetworkActorGenerator();

    boost::shared_ptr<CellPopulationActorGenerator<DIM> > GetCellPopulationActorGenerator();

    /**
     * Update the renderer, this will update the population actor and write output images
     * @param timeStep the curren time step, for annotating output files
     */
    void ResetRenderer(unsigned timeStep=0);

    /**
    * Render the scene
    */
    void Start();

    /**
    * Set the cell population
    * @param pCellPopulation the cell population for rendering
    */
    void SetCellPopulation(boost::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation);

    /**
     * Set the path for output
     * @param rPath the path for output
     */
    void SetOutputFilePath(const std::string& rPath);

    void SetIsInteractive(bool isInteractive);

    void SetSaveAsAnimation(bool saveAsAnimation);

    void SetSaveAsImages(bool saveAsImages);

    void StartInteractiveEventHandler();

    void SetPart(boost::shared_ptr<Part<DIM> > pPart);

    void SetVesselNetwork(boost::shared_ptr<VesselNetwork<DIM> > pNetwork);

    void SetRegularGrid(boost::shared_ptr<RegularGrid<DIM> > pGrid);

    void SetMesh(boost::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh);

};

#endif /* MICROVESSELVTKSCENE_HPP_*/
