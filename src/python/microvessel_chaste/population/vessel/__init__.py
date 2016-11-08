import warnings

# At the moment there are many harmless duplicate registration warnings from boost python. Ignore them until a suitable
# way to avoid duplicate registration during wrapper building is found.
warnings.filterwarnings("ignore")
from _chaste_project_MicrovesselChaste_vessel import *