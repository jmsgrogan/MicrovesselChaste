"""
.. module:: angiogenesis.mesh
   :synopsis: Finite element meshing and regular grid tools.

.. moduleauthor:: James Grogan <grogan@maths.ox.ac.cuk>

"""
import warnings

# At the moment there are many harmless duplicate registration warnings from boost python. Ignore them until a suitable
# way to avoid duplicate registration during wrapper building is found.
warnings.filterwarnings("ignore")

from _chaste_project_MicrovesselChaste_mesh import *
from numpy_tools import *