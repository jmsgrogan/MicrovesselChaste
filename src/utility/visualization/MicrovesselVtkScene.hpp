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

#ifndef MICROVESSELVTKSCENE_HPP_
#define MICROVESSELVTKSCENE_HPP_

#include <vector>
#include "SmartPointers.hpp"
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include <vtkVersion.h>
#if VTK_MAJOR_VERSION > 5
    #include <vtkOggTheoraWriter.h>
    #if VTK_MINOR_VERSION > 0
        #include <vtkAutoInit.h>
        #if VTK_MAJOR_VERSION > 6
        VTK_MODULE_INIT(vtkRenderingOpenGL2);
        #else
        VTK_MODULE_INIT(vtkRenderingOpenGL);
        #endif
        VTK_MODULE_INIT(vtkRenderingFreeType);
    #else
        #define vtkRenderingCore_AUTOINIT 4(vtkInteractionStyle,vtkRenderingFreeType,vtkRenderingFreeTypeOpenGL,vtkRenderingOpenGL)
        #define vtkRenderingVolume_AUTOINIT 1(vtkRenderingVolumeOpenGL)
    #endif
#endif
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

    /**
     * Whether to include orientation axes
     */
    bool mIncludeAxes;

    /**
     * The cell population actor generator
     */
    std::shared_ptr<CellPopulationActorGenerator<DIM> > mpCellPopulationGenerator;

    /**
     * The part actor generator
     */
    std::shared_ptr<PartActorGenerator<DIM> > mpPartGenerator;

    /**
     * The vessel network actor generator
     */
    std::shared_ptr<VesselNetworkActorGenerator<DIM> > mpNetworkGenerator;

    /**
     * The mesh actor generator
     */
    std::shared_ptr<DiscreteContinuumMeshActorGenerator<DIM> > mpDiscreteContinuumMeshGenerator;

    /**
     * The regular grid actor generator
     */
    std::shared_ptr<RegularGridActorGenerator<DIM> > mpGridGenerator;

    /**
     * Scale features using this length. e.g. set to micron if we want
     * to render features in microns
     */
    QLength mLengthScale;

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

    /**
     * Render the current scene and return is as a char array, can be passed
     * into a Python buffer for display.
     * @return the scene as a char array
     */
    vtkSmartPointer<vtkUnsignedCharArray> GetSceneAsCharBuffer();

    /**
     * Return the renderer
     * @return the vtk renderer
     */
    vtkSmartPointer<vtkRenderer> GetRenderer();

    /**
     * @return the part actor generator
     */
    std::shared_ptr<PartActorGenerator<DIM> > GetPartActorGenerator();

    /**
     * @return the mesh actor generator
     */
    std::shared_ptr<DiscreteContinuumMeshActorGenerator<DIM> > GetDiscreteContinuumMeshActorGenerator();

    /**
     * @return the grid actor generator
     */
    std::shared_ptr<RegularGridActorGenerator<DIM> > GetRegularGridActorGenerator();

    /**
     * @return the vessel network actor generator
     */
    std::shared_ptr<VesselNetworkActorGenerator<DIM> > GetVesselNetworkActorGenerator();

    /**
     * @return the cell population actor generator
     */
    std::shared_ptr<CellPopulationActorGenerator<DIM> > GetCellPopulationActorGenerator();

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
    void SetCellPopulation(std::shared_ptr<AbstractCellPopulation<DIM> > pCellPopulation);

    /**
    * Set the part
    * @param pPart the part rendering
    */
    void SetPart(PartPtr<DIM> pPart);

    /**
    * Set the vessel network
    * @param pNetwork the vessel network for rendering
    */
    void SetVesselNetwork(std::shared_ptr<VesselNetwork<DIM> > pNetwork);

    /**
    * Set the grid
    * @param pGrid the grid for rendering
    */
    void SetRegularGrid(std::shared_ptr<RegularGrid<DIM> > pGrid);

    /**
    * Set the mesh
    * @param pMesh the mesh for rendering
    */
    void SetMesh(std::shared_ptr<DiscreteContinuumMesh<DIM> > pMesh);

    /**
     * Set the path for output
     * @param rPath the path for output
     */
    void SetOutputFilePath(const std::string& rPath);

    /**
     * Set run as an interactive window
     * @param isInteractive run as an interactive window
     */
    void SetIsInteractive(bool isInteractive);

    /**
     * Whether to save as an animation
     * @param saveAsAnimation save as an animation
     */
    void SetSaveAsAnimation(bool saveAsAnimation);

    /**
     * Whether to save as images (default)
     * @param saveAsImages save as images
     */
    void SetSaveAsImages(bool saveAsImages);

    /**
     * Start the event handler for window interaction
     */
    void StartInteractiveEventHandler();

};

#endif /* MICROVESSELVTKSCENE_HPP_*/
