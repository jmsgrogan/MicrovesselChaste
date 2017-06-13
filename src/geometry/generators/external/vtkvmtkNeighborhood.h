/*=========================================================================

  Program:   VMTK
  Module:    $RCSfile: vtkvmtkNeighborhood.h,v $
  Language:  C++
  Date:      $Date: 2006/04/06 16:46:43 $
  Version:   $Revision: 1.3 $

  Copyright (c) Luca Antiga, David Steinman. All rights reserved.
  See LICENCE file for details.

  Portions of this code are covered under the VTK copyright.
  See VTKCopyright.txt or http://www.kitware.com/VTKCopyright.htm 
  for details.

     This software is distributed WITHOUT ANY WARRANTY; without even 
     the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR 
     PURPOSE.  See the above copyright notices for more information.

=========================================================================*/
// .NAME vtkvmtkNeighborhood - ..
// .SECTION Description
// ..

#ifndef __vtkvmtkNeighborhood_h
#define __vtkvmtkNeighborhood_h

// JG: Add code to prevent deprecation warnings - June '17
#define _BACKWARD_BACKWARD_WARNING_H 1 //Cut out the vtk deprecated warning
#include "vtkObject.h"
#include "vtkvmtkDataSetItem.h"
//#include "vtkvmtkDifferentialGeometryWin32Header.h"
#include "vtkvmtkWin32Header.h"

class VTK_VMTK_DIFFERENTIAL_GEOMETRY_EXPORT vtkvmtkNeighborhood : public vtkvmtkDataSetItem 
{
public:

  vtkTypeMacro(vtkvmtkNeighborhood,vtkvmtkDataSetItem);

  vtkGetMacro(IsBoundary,bool);

  vtkIdType GetNumberOfPoints() {return this->NPoints;};
  vtkIdType GetPointId(vtkIdType i) {return this->PointIds[i];};

  vtkIdType *GetPointer(vtkIdType i) {return this->PointIds+i;};

  // Description:
  // Build the neighborhood.
  virtual void Build() = 0;

  // Description:
  // Standard DeepCopy method.
  virtual void DeepCopy(vtkvmtkItem *src);

protected:
  vtkvmtkNeighborhood();
  ~vtkvmtkNeighborhood();

  void ResizePointList(vtkIdType ptId, int size);

  vtkIdType NPoints;
  vtkIdType* PointIds;
  bool IsBoundary;

private:
  vtkvmtkNeighborhood(const vtkvmtkNeighborhood&);  // Not implemented.
  void operator=(const vtkvmtkNeighborhood&);  // Not implemented.
};

#endif

