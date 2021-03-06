# imports
import shlex
import math
import numpy as np
import numpy.matlib
import matplotlib.pyplot as plt

def ohigginsRaw2g2o(infoOdomPos, infoOdomAng, infoPointSen, dataDir, dataSkip, dataSize):
    
    # filenames
    inDeadReckon = dataDir + "deadReckoning.dat"
    inMeasurement = dataDir + "measurement.dat"
    outG2O = dataDir + "ohiggins.g2o"
    fg2o = open(outG2O, 'w')
    
    odometry = [line.rstrip('\n') for line in open(inDeadReckon)]
    measurements = [line.rstrip('\n') for line in open(inMeasurement)]

    #print measurements[1]
    
    poseID = 0
    landID = len(odometry)
    j = 1
    count = 0
    x = []
    y = []

    for i in range((len(odometry))):
        odomLine = odometry[i]
        # don't use first line
        if (odomLine[0] == '#' or count > dataSize):
            continue
        # data skip
        if poseID % dataSkip == 0:
            odomWords = shlex.split(odomLine)
            # check not last pose
            if i+dataSkip < len(odometry):
                # odometry
                nextWords = shlex.split(odometry[i+dataSkip])
                x1 = float(odomWords[2])
                y1 = float(odomWords[3])
                a1 = float(odomWords[4])
                x2 = float(nextWords[2])
                y2 = float(nextWords[3])
                a2 = float(nextWords[4])
                dx =  (x2 - x1)*math.cos(a1) + (y2 - y1)*math.sin(a1) #traslacion y rotacion
                dy = -(x2 - x1)*math.sin(a1) + (y2 - y1)*math.cos(a1) #traslacion y rotacion
                dt = ((a2 - a1 + math.pi) % (2*math.pi)) - math.pi
                fg2o.write("EDGE_SE2 " + str(poseID) + " " + str(poseID+dataSkip) + " " + str(dx) + " " +
                str(dy) + " " + str(dt) + " " + str(infoOdomPos) + " 0 0 " + str(infoOdomPos) + " 0 " + str(infoOdomAng) + "\n")
                x.append(x1)
                y.append(y1)
                count = count+1


            #print measurements[j]
            measLine = measurements[j]
            measWords = shlex.split(measLine)            	   
            odomTime = float(odomWords[0]) + ( (float(odomWords[1])*1e-9) - int(float(odomWords[1])*1e-9) )
            measTime = float(measWords[0]) + ( (float(measWords[1])*1e-9) - int(float(measWords[1])*1e-9) )
            while (j < len(measurements) and measTime <= odomTime):
                measLine = measurements[j]
                measWords = shlex.split(measLine)                
                measTime = float(measWords[0]) + ( (float(measWords[1])*1e-9) - int(float(measWords[1])*1e-9) )

                if (measTime == odomTime):
                    #px = float(odomWords[2])
                    #py = float(odomWords[3])
                    
                    #mr = float(measWords[2])
                    #mt = float(measWords[3])
                    
                    #lx = mr*math.cos(mt)
                    #ly = mr*math.sin(mt)              
                    
                    lx = float(measWords[2])
                    ly = float(measWords[3])

                    fg2o.write("EDGE_SE2_XY " + str(poseID) + " " + str(landID) + " " + str(lx) +
                        " " + str(ly) + " " + str(infoPointSen) + " 0 " + str(infoPointSen) + "\n")
                    landID = landID + 1
                    #x.append(lx)
                    #y.append(ly)
                j = j+1
        poseID = poseID + 1

    fg2o.write("FIX " + str(0) + "\n")
    fg2o.close()
    return x,y
