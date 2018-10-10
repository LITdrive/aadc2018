######################################################
# Obstacle in Area                                   #
# Checks if an Obstacle is in given Areas            #
# draw the Areas via the pathPoints.py method        #
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
@Philipp Seidl
"""
######################################################
visualize = False
useMAP = False
is_debugging= False

import json
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal #for 2dconv
from litdrive.zeromq.demo.aadc_geometrics import * #get car geometrics in cm
import scipy.misc
from scipy import ndimage # to rotate local map
#from skimage.draw import line_aa #to draw lines
import pandas as pd

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

if useMAP:
    #### SET UP MAPS
    MAP_X = 30 #in meters
    MAP_Y = 30
    RES = 15 #points per meter
    GPoints = 4 #gaussian points #size of gaussian that will be put in for every measurment
    #TODO adapt to add std of gaussian measurmen right now its around 2cm with RES=20 and GPoints=6
    MAP_D_TYPE= np.int16 # from -32k to +32k
    MAP_BUFF_LEN = 30
    obst_map = np.zeros((MAP_X*RES,MAP_Y*RES,MAP_BUFF_LEN)).astype(dtype=MAP_D_TYPE) #global obstacle map #TODO astype
    MAP_STATIC = np.zeros((MAP_X*RES,MAP_Y*RES,3)) 
    #fist boolean tells you if you have seen that #second if there is an obst #third is movement
    iim = 0 #counter for map updates

    def getSimpleGauss():
        fwhm=GPoints//2
        x = np.arange(0, GPoints, 1, float)
        y = np.arange(0, GPoints, 1, float)[:,np.newaxis]  
        x0 = GPoints//2
        y0 = GPoints//2 
        return (np.exp(-4*np.log(2) * ((x-x0)**2 + (y-y0)**2) / fwhm**2)*100).astype(dtype=MAP_D_TYPE)
    Gau = getSimpleGauss() #just do it once

    def updateMap(dist_compl,x=7,y=15,heading=0, decay=0.6):
        drawFreeWay=True #TODO

        global obst_map, iim
        points = localToGlobal(x,y,heading,dist_compl)
        points*=RES
        Mg = np.zeros(obst_map.shape[:2]).astype(dtype=MAP_D_TYPE)
        M = Mg.copy()
        #M[points.imag.astype(int),points.real.astype(int)]=1
        #GM = signal.convolve2d(M, Gau, boundary='symm', mode='same') #takes too long
        for p in points:
            r=p.imag.astype(int)
            c=p.real.astype(int)
            try:
                if drawFreeWay:#draw line in matrix
                    r0 = int(y*RES)
                    c0 = int(x*RES)

                    lc = [r0,c0,r,c] #will be overritten

                    a0 = np.array(r0+1j*c0)
                    a1 = np.array(r+1j*c)
                    b = a0-a1
                    bn = toComplex(np.abs(b)-2, np.angle(b,deg=True))
                    bn +=a0
                    #lc = [r0,c0,int(bn.real),int(bn.imag)]
                    rr, cc, val = line_aa(*lc) #not really demaning --> 1%
                    M[rr, cc] = np.logical_or(M[rr,cc]>0, val>0) 

                Mg[r-GPoints//2:r+GPoints//2,c-GPoints//2:c+GPoints//2]+=Gau
            except:
                print('out of array')
        Mg[Mg>100]=100 #cap at a 100
        obst_map[:,:,iim%MAP_BUFF_LEN]= M*(-50)+Mg #add to map
        iim+=1
        if ((iim+1)%MAP_BUFF_LEN)==0: #update static map every _ time the buffer has been refilled
            global MAP_STATIC
            # allready seen
            seen_idx = MAP_STATIC[:,:,0]
            mm = obst_map.mean(axis=2)
            now_seen_idx = np.logical_or((mm<-10),(mm>10))

            not_seen_and_now_seen =  (seen_idx==0)*now_seen_idx 
            # if somehting has changed from what you have seen new--> change status to unseen
            has_changed = np.logical_xor( MAP_STATIC[:,:,1], (mm>=0) )*now_seen_idx

            #plt.imshow(MAP_STATIC[:,:,0]*has_changed)
            #plt.show()
            
            # update what you havnt seen 
            MAP_STATIC[:,:,1][not_seen_and_now_seen] = (mm[not_seen_and_now_seen]>0)
            MAP_STATIC[:,:,0][not_seen_and_now_seen] = 1 # change to seen
            MAP_STATIC[:,:,0][has_changed]=0 #change to unseen

            MAP_STATIC[:,:,2]=has_changed
            #MAP_STATIC[seen_idx][:,:,1] += obst_map[seen_idx].mean(2)/100 #old method with probilities
            #MAP_STATIC /= MAP_STATIC.max()


    def plotMap():
        #v1.set_data(M)
        if visualize:
            global iim, MAP_STATIC
            v2.set_data(obst_map[:,:,iim%MAP_BUFF_LEN])
            v3.set_data(obst_map.std(2))
            v4.set_data((MAP_STATIC[:,:,1]*2-1)*MAP_STATIC[:,:,0]) #seen and there = 1; 
            v5.set_data((MAP_STATIC[:,:,2])) #has changed
            #seen and nothing there = -1; # not seen = 0
            plt.draw()
            plt.pause(0.01)

    def getLocalMap(dist_compl):
        """
            # not ready
            computes local map, and returns the map as matrix, 
            as well as the car position (x,y or column/row) in the local map
        """
        sdc=dist_compl*RES
        #clms are real ;)
        #rws are imaginary :D #rows
        map_padd = 1*RES #add a meter
        rws_ofs = abs(sdc.imag.min())+map_padd #offsetX
        rws = abs(sdc.imag.max())+(rws_ofs)
        clms_ofs = abs(sdc.real.min())+map_padd
        clms = abs(sdc.real.max())+(clms_ofs)
        M = np.zeros((np.round(rws+map_padd).astype(int),np.round(clms+map_padd).astype(int))).astype(dtype=MAP_D_TYPE)#empty local map
        Mg = M.copy()
        points =  sdc + np.array([clms_ofs+1j*rws_ofs]) #scale
        #M[points.imag.astype(int),points.real.astype(int)]=10   
        for p in points:
            r=np.round(p.imag).astype(int)
            c=np.round(p.real).astype(int)
            try:
                #draw line in matrix
                lc = [np.round(rws_ofs).astype(int),np.round(clms_ofs).astype(int),r,c]
                rr, cc, val = line_aa(*lc) #not really demaning --> 1%
                M[rr, cc] = np.logical_or(M[rr,cc]>0, val>0) 
                #add gaussian
                Mg[r-GPoints//2:r+GPoints//2,c-GPoints//2:c+GPoints//2]+=Gau
            except:
                print('Error: out of array when calculating the local map',r,c)
        Mg[Mg>100]=100 #cap the gaussian matrix
        car_pos_in_loc_mat = np.array([np.round(clms_ofs).astype(int), np.round(rws_ofs).astype(int)])
        #Mg[car_pos_in_loc_mat[1],car_pos_in_loc_mat[0]]=300 #add car pos
        return M*(-100)+Mg, car_pos_in_loc_mat


    def rotateImage(img, angle, pivot):
        """
            https://stackoverflow.com/questions/25458442/rotate-a-2d-image-around-specified-origin-in-python
            rotation around a so called pivot point w
            (While the image shape is in row-column order,
            pivot is in X-Y or column-row order here. You might want to define it differently.)    
        """
        padX = [img.shape[1] - pivot[0], pivot[0]]
        padY = [img.shape[0] - pivot[1], pivot[1]]
        imgP = np.pad(img, [padY, padX], 'constant')
        imgR = ndimage.rotate(imgP, angle, reshape=False)
        return imgR[padY[0] : -padY[1], padX[0] : -padX[1]]

    def projLocMap2Glob(loc_map,car_pos_in_loc_map,x,y,heading):
        """
            heading in degrees
            not done
        """
        c = int((loc_map.shape[0]**2+loc_map.shape[1]**2)**(1/2)) #pytago
        I = np.zeros((c,c))
        r1 = (c-loc_map.shape[0])//2
        c1 = (c-loc_map.shape[1])//2
        I[r1:r1+loc_map.shape[0],c1:c1+loc_map.shape[1]] = loc_map #place it in the middle

        car_pos_in_imgR = [c1+c//2,r1+c//2]
        imgR = rotateImage(loc_map,heading,car_pos_in_imgR)
        plt.figure()
        plt.imshow(imgR)
        plt.show()

        global obst_map
        #TODO add it in obst_map
        return car_pos_in_imgR#np.zeros(obst_map.shape[:2])[x*RES,y*RES]

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

def processLidar(lidar):
    # processLidar -------------------------------
    l = lidar[0] #amount of lidar elements
    if (l=='NoneType') or (l==0) or (l==None):
        print('lidar not propperly initialized --> I dont get any values from the lidar')
        return (1337, 42), (7331, True) #TODO maybe let it crash
    
    dist = np.asarray( lidar[1::2][:l] )/1000 #in meters
    ang = np.asarray( lidar[2::2][:l] ) # angle is in degrees

    #remove those with 0's and convert it to complex coordinate system
    ldr_compl = toComplex(dist[dist!=0], ang[dist!=0])+LIDAR_LOC_COMP
    return ldr_compl

def processUS(ultraSonic):
    # process UltrasonicSensors --------------------
    us_ts = ultraSonic[0]
    global i
    us_Buff[i%usBuffSize] = np.asarray(ultraSonic)[1::2]/100 # in meters
    i+=1
    us_dist=np.median(us_Buff,axis=0)
    valid_us_idxs = (us_dist!=0)*(us_dist<(US_BEAM_DIM/100)) #and

    valUsDistMult = us_dist[valid_us_idxs]
    valUsAngDeg = US_ANGLES_DEG[valid_us_idxs]
    # TODO --> uggly solution
    valUsDistMult=np.asarray([*valUsDistMult,*valUsDistMult,*valUsDistMult])
    valUsAngDeg = np.asarray([*(valUsAngDeg-6),*valUsAngDeg,*(valUsAngDeg+6)])
    us_compl = toComplex(valUsDistMult,valUsAngDeg)+[*US_LOCS_COMP[valid_us_idxs],*US_LOCS_COMP[valid_us_idxs],*US_LOCS_COMP[valid_us_idxs]]
    return us_compl

def processPOS(pos):
    x = pos[0]
    y = pos[1]
    radius = pos[2]
    speed = pos[3]
    heading = pos[4]
    return x,y,heading

def process(lidar,ultraSonic,position): #TODO remove IMU and Speed
    """
       Process the lidar and US data from Car: to a complex coordinate system
       0 measurments are removed (either too far or too near 
    """
    if lidar is None or ultraSonic is None or position is None:
        return None
    
    x,y,heading=processPOS(position)

    ldr_compl=processLidar(lidar)
    us_compl = processUS(ultraSonic)

    # merge US and Lidar:
    dist_compl = np.append(ldr_compl,us_compl) # count US double --> #TODO split US angle and add 2 points or more

    # project it to global coodrinates:
    dist_compl_global = localToGlobal(x,y,heading,dist_compl)

    # check how many points are in local areas of intrest
    ptsInLocalAOIs=[]
    for ara in localAOIs:
        ptsInLocalAOIs.append(getPointsInArea(dist_compl,ara))


    # check if points are in the global areas
    ptsInGlobalAOIs=[]
    for ara in areas:
        ptsInGlobalAOIs.append(getPointsInArea(dist_compl,ara))

    if useMAP:
        if (i%1)==0: #update every _ samples
            updateMap(dist_compl)
            #print('map updated')

        global iim
        if ((iim+1)%15)==0: #draw ever ... updates
            #locMap, loc_map_car_pos= getLocalMap(dist_compl)
            plotMap()

    if is_debugging:
        plt.figure()
        ldr_compl_gl = localToGlobal(x,y,heading,ldr_compl)
        us_compl_gl = localToGlobal(x,y,heading,us_compl)
        #plt.plot(ldr_compl_gl.real,ldr_compl_gl.imag,'r.')
        #plt.plot(us_compl_gl.real,us_compl_gl.imag,'b.')

        points = dist_compl_global
        isinArea = np.zeros((len(points),areas.shape[1]))
        for rw, p in enumerate(points):
            for clm, a in enumerate(areas): #for every row
                isIn= insideArea(p,a)
                isinArea[rw,clm]=isIn
            clr = 'k'
            if (isinArea[rw,:].sum())>0: 
                clr = 'r'
            #plt.text(p.real,p.imag,clr)
            plt.scatter(p.real,p.imag,c=clr,s=3)

        # draw the car
        carx=np.array([0.48,-0.12,-0.12,0.48,0.48,0.48,0.5])
        cary=np.array([1,1,-1,-1,1,0,0])*CAR_WIDTH/200
        carC=np.array([carx+1j*cary])[0]
        carCg=localToGlobal(x,y,heading,carC)
        plt.plot(carCg.real,carCg.imag,'k')
        plt.plot(localToGlobal(x,y,heading,US_LOCS_COMP).real,localToGlobal(x,y,heading,US_LOCS_COMP).imag,'b*')
        plt.plot(localToGlobal(x,y,heading,LIDAR_LOC_COMP).real,localToGlobal(x,y,heading,LIDAR_LOC_COMP).imag,'r*')
        
        # draw the areas
        for row in range(areas.shape[0]):
            a = areas[row]
            a = np.array([*a, a[0]])
            plt.plot(a.real,a.imag)
            plt.text(a[0].real+0.1, a[0].imag,str(row))
        plt.show()

    # prepare signal out
    sig_outs = []
    for nPiA in [*ptsInLocalAOIs, *ptsInGlobalAOIs]:
        sig_outs.append( (1337, nPiA) )

    if len(sig_outs)==1:
        return sig_outs

    return (*sig_outs,)

def insideArea(point, area):
    """
    Return True if a coordinate (x, y) is inside a polygon (area) defined by
    a list of verticies [(x1, y1), (x2, x2), ... , (xN, yN)].
    Reference: http://www.ariel.com.au/a/python-point-int-poly.html
    """
    x=point.real
    y=point.imag
    n = len(area)
    inside = False
    p1x = area[0].real
    p1y = area[0].imag
    for i in range(1, n + 1):
        p2x = area[i % n].real
        p2y = area[i % n].imag
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside

def getPointsInArea(dist_compl,area):
    ctr = 0
    for rw, point in enumerate(dist_compl):
        isIn= insideArea(point,area)
        if isIn: ctr+=1
    return ctr

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
    shift_dist_compl= dist_compl-( centerBoxCoordinate-np.array([box_height/2+0j]) )
    #plt.plot(dist_compl.real,dist_compl.imag,'r.')
    # now look in the box in front of you
    obstacleIdx = (shift_dist_compl.real<box_height)*(abs(shift_dist_compl.imag)<((box_width)))
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
    #load the local areas:
    df = pd.read_csv('litdrive/zeromq/phil/localAOIs.csv',header=None)
    a = df.values
    pts = a[:,1]+1j*a[:,2]
    pts-=pts[0] #everything is relative to the car pos
    # the first index is front tip of the car --> offset to hinteraxn (backaxle)
    # has to be shifted by:
    pts+=np.array([CAR_HEIGHT/200+0j])+SHIFT #front spizzal to hinteraxn
    localAOIs = pts[1:].reshape(4,-1) #make sure it is devidable by 4
    if is_debugging:
        plt.plot(pts.real,pts.imag)

    #load ares to check:
    df = pd.read_csv('litdrive/zeromq/phil/areas.csv',header=None) #add Property
    a = df.values
    ofs = 0
    areas = a[:,1]+1j*a[:,2]
    areas=areas.reshape(4,-1)
    #TODO append if less then 20


    # open a server for the filter
    NUM_AREAS = 20 #TODO adjust
    tsv=[]
    for i in range(NUM_AREAS):
         tsv.append("tSignalValue")
    zmq = ZmqServer("tcp://*:5565",
                    ["tLaserScannerData","tUltrasonicStruct","tPosition"],
                    tsv)

    if useMAP:
        if visualize:
            # get an empty interactive view for the image
            plt.ion()
            plt.figure(figsize=(5,5)) #figsize=(10, 10)
            #plt.subplot(211)
            #plt.title('local map')
            #v1 = plt.imshow(np.zeros((10,10)))#obst_map.sum(2))
            #plt.colorbar()
            plt.subplot(221)
            v2 = plt.imshow(obst_map.mean(2),vmin=-100,vmax=100,cmap='bwr')
            plt.colorbar()
            plt.subplot(222)
            plt.title('std-map')
            v3 = plt.imshow(obst_map.std(2),vmin=0,vmax=60,cmap='hot')
            plt.colorbar()
            plt.subplot(223)
            plt.title('static-map')
            v4 = plt.imshow(MAP_STATIC[:,:,1],vmin=-1,vmax=1,cmap='binary')
            plt.colorbar()
            plt.subplot(224)
            plt.title('movement-map')
            v5 = plt.imshow(MAP_STATIC[:,:,2],vmin=0,vmax=1,cmap='hot')
            plt.colorbar()
    try:
        zmq.connect()
        zmq.run(process, return_dict=False)

    finally:
        zmq.disconnect()
