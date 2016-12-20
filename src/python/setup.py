"""Copyright (c) 2005-2015, University of Oxford.
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

from setuptools import setup, Distribution,find_packages

class BinaryDistribution(Distribution):
    def is_pure(self):
        return False
    
    def has_ext_modules(self):
        return True

setup(
    name = "microvessel_chaste",
    version = "3.4.2",
    packages = find_packages(),
    #install_requires = ['petsc4py==3.7', 'matplotlib', 'numpy'],
    package_data={
        'microvessel_chaste': ['_chaste_project_MicrovesselChaste_preload.so', 
                   'geometry/_chaste_project_MicrovesselChaste_geometry.so',
                   'mesh/_chaste_project_MicrovesselChaste_mesh.so', 
                   'pde/_chaste_project_MicrovesselChaste_pde.so', 
                   'utility/_chaste_project_MicrovesselChaste_utility.so', 
                   'population/vessel/_chaste_project_MicrovesselChaste_vessel.so', 
                   'population/cell/_chaste_project_MicrovesselChaste_cell.so', 
                   'visualization/_chaste_project_MicrovesselChaste_visualization.so', 
                   'simulation/_chaste_project_MicrovesselChaste_simulation.so', 
                   'simulation/_chaste_project_MicrovesselChaste_flow.so', 
                   'simulation/_chaste_project_MicrovesselChaste_angiogenesis.so',],},
      
    data_files = [('tutorials', ['doc/tutorials/TestPythonOffLatticeAngiogenesisLiteratePaper.ipynb', 
                                'doc/tutorials/TestPythonLatticeBasedAngiogenesisTutorial.ipynb',
                                'doc/tutorials/TestPythonBuildVesselNetworkTutorial.ipynb',
                                'doc/tutorials/TestPythonBiologicalNetworkLiteratePaper.ipynb',
                                'doc/tutorials/bio_original.vtp',])],
      
    include_package_data=True,
    zip_safe = False,

    # Project Metadata
    author = "Chaste Team, University of Oxford",
    author_email = "grogan@maths.ox.ac.uk",
    description = "Python bindings for the Chaste project",
    license = "BSD",
    keywords = "cancer developmental biology electrophysiology scientific",

    classifiers=[
        'Development Status :: 1 - Planning',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering',
        'Operating System :: Unix',  
        'License :: OSI Approved :: BSD License',
        'Programming Language :: Python :: 2.7',
    ],

    distclass=BinaryDistribution
)
