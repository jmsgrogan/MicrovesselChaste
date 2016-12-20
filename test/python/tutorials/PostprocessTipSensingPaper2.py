
"""Copyright (c) 2005-2016, University of Oxford.
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
"""

import numpy as np
import csv
import matplotlib as mpp
import matplotlib.pyplot as plt
import vtk


        
if __name__ == '__main__':
    font = {'family' : 'normal',
        'size'   : 14}

    mpp.rc('font', **font)
    
    work_dir = "/home/grogan/test/Python/TestTipSensingPaper_ThesisParams/"
    
    sample_times = [0, 145, 289, 421]
    num_runs = 1
    batch = "3D_Moore_Sense_b"
    
    exp_data_postitions =  [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0] 
    exp_data_day_3 =  [1.1, 1.1, 1.0, 0.6, 0.1, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0] 
    exp_data_day_5 =  [1.1, 1.0, 0.95, 1.0, 0.95, 0.75, 0.28, 0.1, 0.0, 0.0, 0.0] 
    
    aj_data_day_3 = []
    aj_data_day_5 = []
    
    for eachTime in sample_times:
        fig, ax = plt.subplots()
        ax.set_xlim([-0.2,1.0])
        ax.set_ylim([0.0,4.25])
        ax.set_xlabel("Location (mm)")
        ax.set_ylabel("Vessel Length Density x 2.e-8 m/m$^3$")
        locations = np.linspace(-0.2, 1.04, (1240/20)+1)
        
        run_densities = []
        for eachRun in range(num_runs):
            run_dir = work_dir+batch + "/Run_" + str(eachRun) + "/"
            filename = run_dir + "VesselNetwork_inc_" + str(eachTime) + ".vtp"
            
            # Read in vessel network
            reader = vtk.vtkXMLPolyDataReader()
            reader.SetFileName(filename)
            reader.Update()
            network_polydata = reader.GetOutput()
            
            # Bin the points on the regular grid
            grid_spacing = 20.0
            grid_extents = [int(2000.0/grid_spacing)+1, int(1240.0/grid_spacing)+1, int(100.0/grid_spacing), +1]
            nodes_list_x_y = []
            for idx in range(grid_extents[0]*grid_extents[1]):
                nodes_list_x_y.append([])
            for idx in range(network_polydata.GetNumberOfPoints()):
                grid_loc_x_y = [int(network_polydata.GetPoint(idx)[0]/grid_spacing), 
                                int(network_polydata.GetPoint(idx)[1]/grid_spacing)]
                grid_index = grid_loc_x_y[0] + grid_extents[0]*grid_loc_x_y[1]
                nodes_list_x_y[grid_index].append(idx)
                
            # Get max num seg per x,y point
            segment_numbers = []
            for gridPoint in nodes_list_x_y:
                max_num_seg = 0
                for eachNode in gridPoint:
                    cellIds = vtk.vtkIdList()
                    network_polydata.GetPointCells(eachNode, cellIds)
                    if cellIds.GetNumberOfIds() > max_num_seg:
                        max_num_seg = cellIds.GetNumberOfIds()
                segment_numbers.append(max_num_seg)
            segment_numbers = np.array(segment_numbers)
                
            # Get Densities
            densities = []
            for eachYLoc in range(grid_extents[1]):
                x_indices = range(eachYLoc*grid_extents[0], (eachYLoc+1)*grid_extents[0])
                local_seg_numbers = segment_numbers[x_indices]
                densities.append((np.sum(local_seg_numbers)*10.0*1e12/(20.0*20.0*float(grid_extents[0])*100.0))/(2.e8))
            run_densities.append(densities)
    
        run_densities = np.array(run_densities)
        ax.plot(locations, run_densities[0], 
                label = "Predicted", color = 'red', lw=2.0)
        #ax.plot(locations, (run_densities[0]+run_densities[1])/2.0, label = "Predicted", color = 'red', lw=2.0)
        ax.plot(exp_data_postitions, exp_data_day_3, label = "Expt3", color = 'green', lw=2.0)
        ax.plot(exp_data_postitions, exp_data_day_5, label = "Expt5", color = 'green', lw=2.0)
        ax.set_aspect(0.2)
        ax.legend()
        fig.savefig('/home/grogan/density_new_params_'+str(eachTime)+'.png') 
