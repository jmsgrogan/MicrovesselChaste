/*=========================================================================

  Program:   VMTK
  Module:    $RCSfile: vtkvmtkPolyDataManifoldStencil.h,v $
  Language:  C++
  Date:      $Date: 2006/04/06 16:46:44 $
  Version:   $Revision: 1.4 $

  Copyright (c) Luca Antiga, David Steinman. All rights reserved.
  See LICENCE file for details.

  Portions of this code are covered under the VTK copyright.
  See VTKCopyright.txt or http://www.kitware.com/VTKCopyright.htm 
  for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
// .NAME vtkvmtkPolyDataManifoldStencil - ..
// .SECTION Description
// ..

// Modified JGrogan May 17: Add 'using' to avoid compiler warnings for hidden virtual methods

#ifndef __vtkvmtkPolyDataManifoldStencil_h
#define __vtkvmtkPolyDataManifoldStencil_h

#include "vtkObject.h"
#include "vtkvmtkStencil.h"
#include "vtkPolyData.h"
#include "vtkvmtkWin32Header.h"

class VTK_VMTK_DIFFERENTIAL_GEOMETRY_EXPORT vtkvmtkPolyDataManifoldStencil : public vtkvmtkStencil 
{
public:

  vtkTypeMacro(vtkvmtkPolyDataManifoldStencil,vtkvmtkStencil);

  vtkGetMacro(Area,double);

  // Description:
  // Build the stencil.
  virtual void Build();

  virtual void ComputeArea();
  virtual void ScaleWithArea() = 0;

  using vtkvmtkStencil::DeepCopy; // Added by JGrogan May 17 to avoid compiler warnings for hidden virtual methods

  void DeepCopy(vtkvmtkPolyDataManifoldStencil *src);

  vtkGetMacro(UseExtendedNeighborhood,int);
  vtkSetMacro(UseExtendedNeighborhood,int);
  vtkBooleanMacro(UseExtendedNeighborhood,int);
  
protected:
  vtkvmtkPolyDataManifoldStencil();
  ~vtkvmtkPolyDataManifoldStencil() {};

  void ScaleWithAreaFactor(double factor);

  double Area;

  int UseExtendedNeighborhood;
  
private:
  vtkvmtkPolyDataManifoldStencil(const vtkvmtkPolyDataManifoldStencil&);  // Not implemented.
  void operator=(const vtkvmtkPolyDataManifoldStencil&);  // Not implemented.
};

#endif

