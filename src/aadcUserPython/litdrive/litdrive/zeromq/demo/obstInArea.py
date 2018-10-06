######################################################
# Obstacle in Area                                   #
# Checks if an Obstacle is in a given Area           #
#
"""
The coordinate system:
            ^   x-axis   / 0degrees --> Re-Axis
            |
            |
      ______|lidar__
      |     |      |
     |  |   |    |  |
      |     |      |
      |     |      |
      |     |      |
      |     |0     |
     |--|--------|--|--->  y-axis  / 270degrees  --> Im-Axis
      |_____|______|
            |
where 0 is the middle of the hinteraxn ;)
"""
######################################################

import json
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal #for 2dconv
from litdrive.zeromq.demo.aadc_geometrics import * #get car geometrics in cm 
# those are measured in the middle and have to be shifted by 36cm in Real axis
# constants have to be devided by 100 --> they are in cm
SHIFT = np.array([0.18+0j]) #TODO

from ..server import ZmqServer

##### LIDAR-Info:
# scan-rate ~10Hz (dep. on amount of samples)
# sample-freq. 4000Hz
# distance range: 0.15 - 12 meters
LIDAR_LOC_COMP = np.array(LS_XPOS+1j*LS_YPOS)/100+SHIFT
MIN_MEASURM = 0.1
MAX_MEASURM = 12

##### Ultrasonic-Info:
# tSideLeft,tSideRight,tRearLeft,tRearCenter,tRearRight
# sample-freq. 40Hz
# distance range: 0.02 - 4 meters
# measuring Angle: <15 degrees
MIN_MEASURM_US = 0.02
MAX_MEASURM_US = 4
US_LOCS_COMP = np.array([
(US_SIDE_LEFT_XPOS+1j*US_SIDE_LEFT_YPOS),
(US_SIDE_RIGHT_XPOS+1j*US_SIDE_RIGHT_YPOS),
(US_REAR_LEFT_XPOS+1j*US_REAR_LEFT_YPOS),
(US_REAR_CENTER_XPOS+1j*US_REAR_CENTER_YPOS),
(US_REAR_RIGHT_XPOS+1j*US_REAR_RIGHT_YPOS)
])/100+SHIFT

US_ANGLES_DEG = np.array([US_SIDE_LEFT_ZROT,US_SIDE_RIGHT_ZROT,US_REAR_LEFT_ZROT,US_REAR_CENTER_ZROT,US_REAR_RIGHT_ZROT])

usBuffSize = 9
i=0
us_Buff = np.ones((usBuffSize,5))

def toComplex(distances, angles_in_degree):
    angle_radians = (angles_in_degree*np.pi/180)
    return distances*np.e**(-1j*angle_radians)# turned cartesian coordinates

def localToGlobal(x,y,heading,local_compl):
    """
        Calculates the global points in a coordinate system given x,y and heading of the car
    """
    # heading is 0 at 0 degrees
    ang_deg = (np.angle(local_compl, deg=True)+heading)
    local_turned_compl = toComplex(np.abs(local_compl),ang_deg)
    # now add x and y
    return np.array([x+1j*y])+local_turned_compl

def globalToLocal(x,y,heading,global_compl):
    """
        Calculates the local point given a global point
    """
    local_compl = global_compl-np.array([x+1j*y]) #subtract x and y
    ang_deg = (np.angle(local_compl, deg=True)-heading) #rotate
    return toComplex(np.abs(local_compl),ang_deg) #and calc0

MAP_X = 30
MAP_Y = 30
RES = 20
obst_map = np.zeros((MAP_X*RES,MAP_Y*RES))
GPoints = 6 #gaussian points

def updateMap(ldr_compl,x=7,y=15,heading=0, decay=0.6):
    points = localToGlobal(x,y,heading,ldr_compl)
    points*=RES
    global obst_map
    M = np.zeros(obst_map.shape)
    M[points.imag.astype(int),points.real.astype(int)]=1
    #GM = signal.convolve2d(M, Gau, boundary='symm', mode='same')
    for p in points:
        r=p.imag.astype(int)
        c=p.real.astype(int)
        try:
            M[r-GPoints//2:r+GPoints//2,c-GPoints//2:c+GPoints//2]+=Gau
        except:
            print('out of array')
    obst_map+=M


def getSimpleGauss():
    fwhm=GPoints//2
    x = np.arange(0, GPoints, 1, float)
    y = np.arange(0, GPoints, 1, float)[:,np.newaxis]  
    x0 = GPoints//2
    y0 = GPoints//2 
    return np.exp(-4*np.log(2) * ((x-x0)**2 + (y-y0)**2) / fwhm**2)
Gau = getSimpleGauss()

def getGauss(point):
    print(point)
    fwhm=RES//4
    x = np.arange(0, MAP_Y*RES, 1, float)
    y = np.arange(0, MAP_X*RES, 1, float)[:,np.newaxis]  
    x0 = point[0]
    y0 = point[1] 
    return np.exp(-4*np.log(2) * ((x-x0)**2 + (y-y0)**2) / fwhm**2)

import time
def plotMap():
    #plt.figure()
    global obst_map
    plt.imshow(obst_map)
    plt.show()
    time.sleep(0.1)
    plt.close()

def process(imu,lidar,speed,ultraSonic): #TODO remove IMU and Speed
    """
       Process the lidar and US data from Car: to a complex coordinate system
       0 measurments are removed (either too far or too near 
    """
    # processLidar -------------------------------
    l = lidar[0] #amount of lidar elements
    if (l==None) or (l==0):
        print('lidar not propperly initialized --> I dont get any values from the lidar')
        return (1337, 42), (7331, True) #TODO maybe let it crash
    
    dist = np.asarray( lidar[1::2][:l] )/1000 #in meters
    ang = np.asarray( lidar[2::2][:l] ) # angle is in degrees

    #remove those with 0's and convert it to complex coordinate system
    ldr_compl = toComplex(dist[dist!=0], ang[dist!=0])+LIDAR_LOC_COMP

    # process UltrasonicSensors --------------------
    us_ts = ultraSonic[0]
    global i
    us_Buff[i%usBuffSize] = np.asarray(ultraSonic)[1::2]/100 # in meters
    i+=1
    us_dist=np.median(us_Buff,axis=0)
    valid_us_idxs = (us_dist!=0)*(us_dist<(US_BEAM_DIM/100)) #and

    valUsDistMult = us_dist[valid_us_idxs]
    valUsAngDeg = US_ANGLES_DEG[valid_us_idxs]
    valUsDistMult=np.asarray([*valUsDistMult,*valUsDistMult,*valUsDistMult])
    valUsAngDeg = np.asarray([*(valUsAngDeg-6),*valUsAngDeg,*(valUsAngDeg+6)])
    us_compl = toComplex(valUsDistMult,valUsAngDeg)+[*US_LOCS_COMP,*US_LOCS_COMP,*US_LOCS_COMP]

    # merge US and Lidar:
    dist_compl = np.append(ldr_compl,us_compl) # count US double --> #TODO split US angle and add 2 points or more

    debug=False
    if debug:
        plt.figure()
        plt.plot(ldr_compl.real,ldr_compl.imag,'r.')
        plt.plot(us_compl.real,us_compl.imag,'b.')
        # draw the car
        carx=[0.48,-0.12,-0.12,0.48,0.48,0.48,0.5]
        cary=np.array([1,1,-1,-1,1,0,0])*CAR_WIDTH/200
        plt.plot(carx,cary,'k')
        plt.plot(US_LOCS_COMP.real,US_LOCS_COMP.imag,'b*')
        plt.plot(LIDAR_LOC_COMP.real,LIDAR_LOC_COMP.imag,'r*')
        plt.ylim((-4,4))
        plt.xlim((-4,4))
        plt.show()

    centerBoxCoordinate=np.array([1+3j])
    box_width=0.5
    box_height=0.5
    ob_found = checkObstacles(dist_compl,centerBoxCoordinate,box_width,box_height,threshold=2)
    print(ob_found)

    updateMap(dist_compl)
    #plotMap()

    signal_out = (1337, 42)
    bool_out = (7331, False) #TODO timestamp????

    return signal_out, bool_out

def checkObstacles(dist_compl, centerBoxCoordinate, box_width, box_height, threshold=2):
    """
        Calculates if Obstacles are in a given area
        Input: -lidar and US in complex coordinate system was well
               -centerBoxCoordinate from car coordinate system as complex number
               -box_width #imag-part ;)
               -box_height
        Everything in meters please.. we are not babaric and like to use SI-Units ;)
        Note that US-data counts as 1 datapoints -> that's why it sould be added twice to dist_compl
        Also make sure that there are no 0's in your dist_compl
        Returns (amount of lidar points within maxLen)>threshold
        for the precise calculation ask ph.seidl92@gmail.com ;)
        ain't nobody got time for documentation
    """
    # move the coordinate system to the center + box_height/2
    #plt.plot(dist_compl.real,dist_compl.imag,'g.')
    dist_compl-= ( centerBoxCoordinate-np.array([box_height/2+0j]) )
    #plt.plot(dist_compl.real,dist_compl.imag,'r.')
    # now look in the box in front of you
    obstacleIdx = (dist_compl.real<box_height)*(abs(dist_compl.imag)<((box_width)))
    #plt.show()
    return sum(obstacleIdx)>threshold

def checkObstaclesAhead(ldr_compl,tireAngle, maxLen=0.3,threshold=2):
    """
        Calculates if Obstacles are ahed based on lidar data and tireAngle
        Searches within CAR_WIDTH and maxLen in the by tireAngle roated coordinate system
        Returns (amount of lidar points within maxLen)>threshold
        for the precise calculation ask ph.seidl92@gmail.com ;)
        ain't nobody got time for documentation
    """
    #within the car-width and the maxLen
    # at 45 degrees shift real for 0.05m
    madeUpHeuristic = tireAngle*0.07/45 #shifts real-axis dependent on tire angle
    madeUpHeuristic2= abs(tireAngle*0.14/45) #at 45degrees append CAR_WIDTH with 0.15m
    obstacleIdx = (ldr_compl.imag<maxLen)*(abs(ldr_compl.real+madeUpHeuristic)<((CAR_WIDTH/100+madeUpHeuristic2)/2))
    if is_debugging:
        plt.plot(ldr_compl.real,ldr_compl.imag,'.')
        plt.show()
        print(sum(obstacleIdx))
    return sum(obstacleIdx)>threshold

if __name__ == "__main__":
    # open a server for the filter
    zmq = ZmqServer("tcp://*:5556",
                    ["tInerMeasUnitData", "tLaserScannerData","tSignalValue","tUltrasonicStruct"],
                    ["tSignalValue", "tBoolSignalValue"])

    try:
        zmq.connect()
        zmq.run(process, return_dict=False)

    finally:
        zmq.disconnect()
