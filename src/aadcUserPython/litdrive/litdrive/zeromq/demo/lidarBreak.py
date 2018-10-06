######################################################
# PLEASE COPY THIS TEMPLATE AND MAKE YOUR OWN FILTER #
######################################################

import json
import numpy as np
import matplotlib.pyplot as plt

from ..server import ZmqServer
is_debugging = False

##### LIDAR-Info:
# scan-rate ~10Hz (dep. on amount of samples)
# sample-freq. 4000Hz
#
# distance range: 0.15 - 12 meters
# (I even measured samples at dist of 0.12 ;)

##### IMU-Info:
# 3-axis, x, y, z
# in the following order: ui32ArduinoTimestamp, A_x, A_y A_z, G_x, G_y, G_z, M_x, M_y, M_z

# constants:
CARWIDTH = 0.32 #meters measured
LIDAR_TO_FRONT = 0.06 # distance from lidar to most front point where impact would accure
LIDAR_TO_BACK_AXIS = 0.46 #lidar to our car coordinate system offset
MIN_MEASURM = 0.1
MAX_MEASURM = 12
MAX_TIRE_ANGLE = 45 #degrees #TODO check

BREMSWEG = 0.3
SAFETY = 0.2
# obstacle search distance = BREMSWEG+SAFETY

def process(lidar,wheelSteeringPercent,speed):
    """
       Process the lidar data from Car: to a complex coordinate system
       0 measurments are removed (either too far or too near (lidar distance range: 0.15 - 12 meters)
    """
    #TODO processSpeed ---------------------------
    speed = speed[1]
    # processTireAngle ---------------------------
    # ranges from -MAX_TIRE_ANGLE to MAX_TIRE_ANGLE
    # -100 is voi links einschlogn --> +45degrees
    tireEinschlog = wheelSteeringPercent[1] #TODO test that stuff
    tireAngle = -tireEinschlog/100*MAX_TIRE_ANGLE
    # processLidar -------------------------------
    l = lidar[0] #amount of lidar elements
    dist = np.asarray( lidar[1::2][:l] )/1000 #in meters
    ang = np.asarray( lidar[2::2][:l] ) # angle is in degrees

    no_sig_idx = dist==0 #remove those with 0's (or <MIN_MEASURM??)
    #transform to complex coordinate system
    # coordinate system is: x-Axis is along car; z-Axis pointing up 
    angle_radians = (((ang[~no_sig_idx]-90+tireAngle)))*np.pi/180
    # roatete polar coordinate system by tireAngle and 90 degrees to have
    # real axis parallel to tire-axis
    ldr_compl = dist[~no_sig_idx]*np.e**(-1j*angle_radians)# turned cartesian coordinates
    #print(ldr_compl) 
    #TODO find out at which distance to stop!!! right now 40cm
    #TODO adjust maxLen by scaling with (1+speed)*scaling
    is_obst_ahead = checkObstaclesAhead(ldr_compl,tireAngle,maxLen=BREMSWEG+SAFETY,threshold=3)
    print('\r Obst. ahead (tireAngle=',tireAngle,'):',is_obst_ahead,end='  ')

    signal_out = (1337, 42)
    bool_out = (7331, is_obst_ahead) #TODO timestamp????
    return signal_out, bool_out

def checkObstaclesAhead(ldr_compl,tireAngle, maxLen=0.3,threshold=2):
    """
        Calculates if Obstacles are ahed based on lidar data and tireAngle
        Searches within CARWIDTH and maxLen in the by tireAngle roated coordinate system
        Returns (amount of lidar points within maxLen)>threshold
        for the precise calculation ask ph.seidl92@gmail.com ;)
        ain't nobody got time for documentation
    """
    #within the car-width and the maxLen
    # at 45 degrees shift real for 0.05m
    madeUpHeuristic = tireAngle*0.07/45 #shifts real-axis dependent on tire angle
    madeUpHeuristic2= abs(tireAngle*0.14/45) #at 45degrees append CARWIDTH with 0.15m
    obstacleIdx = (ldr_compl.imag<maxLen)*(abs(ldr_compl.real+madeUpHeuristic)<((CARWIDTH+madeUpHeuristic2)/2))
    if is_debugging:
        plt.plot(ldr_compl.real,ldr_compl.imag,'.')
        #plt.xlim((-0.5,0.5))
        #plt.ylim((0,1))
        #print(min(ldr_compl.imag[obstacleIdx]))
        #plt.plot((ang+90)%360,dist,'.')
        plt.show()
        print(sum(obstacleIdx))
    return sum(obstacleIdx)>threshold

if __name__ == "__main__":
    # open a server for the filter
    zmq = ZmqServer("tcp://*:5557",
                    ["tLaserScannerData","tSignalValue","tSignalValue"],
                    ["tSignalValue", "tBoolSignalValue"])
    try:
        zmq.connect()
        zmq.run(process, return_dict=False)
    finally:
        zmq.disconnect()
