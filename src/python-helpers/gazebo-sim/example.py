#!/usr/bin/python

'''
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
'''

#imports
import subprocess
import sys
import time
import os
from time import gmtime, strftime
sys.path.append('../commons')
sys.path.append('./data/Clay')
#from slamFunctions import *
#from g2o2lab import *
from OHigginsRaw2g2o import *

import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt
    
# variables
#g2oIterations = 15
g2oIterations = 15
xi = 0
#infoOdomPos = 8000
#infoOdomAng = 100000
#infoOdomAng = 10000

#infoOdomPos = 0.05
#infoOdomAng = 0.01

#infoPointSen = 5
#infoPointSen = 0.01

#dataSkip = 150
dataSkip = 2
interOpt = 500
#dataSize = 100000
dataSize = 10000
#disTest = 5
disTest = 2
kernelWidth = 1
poseSkip = 10

infoOdomPos = 20
infoOdomAng = 20
infoPointSen = 10

# compile
buildPath = "../../graphSLAM/build/"
subprocess.call(["make", "-C", buildPath])

# run g2o tests
# start_time = time.time()

# # paths
# #dataPath = "data/ROS/ROS.g2o"
dataPath = "data/Clay/ohiggins.g2o"
# #dataPath = "data/Victoria Park/victoria.g2o"

# #dataPath = "data/Clay/test1.g2o"
dataName = os.path.splitext(os.path.basename(dataPath))[0]
dataDir = os.path.dirname(dataPath) + "/"
# gtPath = dataDir + "gt.g2o"
# guessOutPath = "res/initialGuessOut_" + dataName + ".g2o"
# resPath = "res/optimized_" + dataName + ".g2o"
# figPath = "res/res_" + dataName

x, y = ohigginsRaw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize)

plt.plot(x, y, 'r-')
plt.ylim((-10,15))
plt.xlim((-5,22))
plt.ylabel('some numbers')
plt.show()
                 