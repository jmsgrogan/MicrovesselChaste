#! /opt/local/bin/python

import numpy
import matplotlib.pyplot as plt
import os

rootDir = os.environ.get('CHASTE_TEST_OUTPUT')
filename = rootDir + 'Vasculature/StructuralAdaptationAlgorithm/TestAlarcon03MechanicalStimulusCalculator/ShearStressVsPressure.dat'

file = open(filename,'r')

pressure = []
tau_p = []

for line in file.readlines():
    radii.append(float(line.split(' ')[0]))
    viscosity15.append(float(line.split(' ')[1].replace('\n','')))

file.close()

plt.rc('text', usetex=True)
plt.rc('font', family='serif')

fig,ax = plt.subplots(figsize=(15,10))
points1, = ax.loglog(pressure,tau_p,'-k',linewidth=2.0)

plt.xlabel('Pressure (mmHg)',fontsize=20)
plt.ylabel('$\tau_p$',fontsize=20)

plt.show()
