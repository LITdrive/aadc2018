import numpy as np
import scipy.integrate
from enum import IntEnum
import pickle


class RoadDecisions(IntEnum):
    NEXT=0
    LEFT=1
    STRAIGHT=2
    RIGHT=3
    MERGE=4
    INVALID=5
    END=6
    MAX=6


class RoadList:
    def __init__(self):
        self.lanes=dict()
        self.roads=dict()
        self.predecessors=dict()
        self.successors=dict()
        return

    def addLaneElement(self, element, id, predecessor_list, successor_list):
        if(not issubclass(type(element), LaneElement)):
            raise Exception("Element is not a Object with Base-class LaneElement!")        
        self.lanes[id]=element
        self.predecessors[id]=predecessor_list
        self.successors[id]=successor_list

    def removeLaneElement(self, id):
        if(id in self.lanes):
            retval=self.lanes[id]
            self.lanes[id]=None
            return retval
        return None
    
    def addRoad(self, element, road_id):
        #if(isinstance(element, RoadElement)==False):
        #    raise Exception("Element is not an RoadElement object!")
        self.roads[road_id]=element

    def setLanePredecessorDict(self, lane_id, pre_dict):
        #if(RoadDecisions.NEXT in pre_dict and len(pre_dict)!=1):
        #    raise Exception("ERROR: predecessor dict is malformed! (has a NEXT and other keys!")
        if(lane_id not in self.lanes):
            raise Exception("ERROR: tried adding predecessor dict for lane that does not exist!")
        self.predecessors[lane_id]=pre_dict

    def setLaneSuccessorDict(self, lane_id, suc_dict):
        #if(RoadDecisions.NEXT in suc_dict and len(suc_dict)!=1):
        #    raise Exception("ERROR: successor dict is malformed! (has a NEXT and other keys!")
        if(lane_id not in self.lanes):
            raise Exception("ERROR: tried adding successor dict for lane that does not exist!")
        self.successors[lane_id]=suc_dict

    def saveToFile(self, filename):
        try:
            f=open(filename, "wb")
            dump=pickle.dumps(self)
            f.write(dump)
            f.close()
        except Exception as e:
            print("ERROR: Writing to \"{}\" failed: \"{}\"!".format(filename,e))
            return False
        return True

    def loadFromFile(self, filename):
        try:
            f=open(filename, "rb")
            loaded=pickle.load(f)
            f.close()
        except Exception:
            return False
        if(not isinstance(loaded, RoadList)):
            print("ERROR: Loading from Pickle failed, not the right type (is \"{}\")!\n{}".format(type(loaded),loaded))
            return False
        self.lanes=loaded.lanes
        self.roads=loaded.roads
        self.successors=loaded.successors
        self.predecessors=loaded.predecessors
        return True

class RoadElement:
    def __init__(self):
        self.lane_0_outer=None
        self.lane_0_inner=None
        self.lane_1_inner=None
        self.lane_1_outer=None
        pass

    # Add a Lane to the class, identified by the original lane id from opendrive and the newly given id.
    # returns true if the add worked or false if it failed!
    def addLane(self, new_lane_id, original_lane_id):
        if(original_lane_id==-2):
            self.lane_0_outer=new_lane_id
        elif(original_lane_id==-1):
            self.lane_0_inner=new_lane_id
        elif(original_lane_id==1):
            self.lane_1_inner=new_lane_id
        elif(original_lane_id==2):
            self.lane_1_outer=new_lane_id
        else:
            return False
        return True

    def getLaneByOpenDriveID(self, original_lane_id):
        if(original_lane_id==-2):
            return self.lane_0_outer
        elif(original_lane_id==-1):
            return self.lane_0_inner
        elif(original_lane_id==1):
            return self.lane_1_inner
        elif(original_lane_id==2):
            return self.lane_1_outer
        else:
            return None        

    def __str__(self):
        retval=["RoadElement:"]
        if(self.lane_1_outer is not None and self.lane_1_inner is not None):
            retval.append(str(self.lane_1_outer))
            retval.append(":")
            retval.append(str(self.lane_1_inner))
        elif(self.lane_1_inner is not None):
            retval.append(str(self.lane_1_inner))
        elif(self.lane_1_outer is not None):
            retval.append(str(self.lane_1_outer))
        retval.append("|")
        if(self.lane_0_outer is not None and self.lane_0_inner is not None):
            retval.append(str(self.lane_0_outer))
            retval.append(":")
            retval.append(str(self.lane_0_inner))
        elif(self.lane_0_inner is not None):
            retval.append(str(self.lane_0_inner))
        elif(self.lane_0_outer is not None):
            retval.append(str(self.lane_0_outer))

        return " ".join(retval)



class LaneElement:

    def getPixelPointList(self, px_per_m, points_per_m):
        raise NotImplementedError("This is the abstract base class. No functions implemented!")

    def calcArcLength(self):
        raise NotImplementedError("This is the abstract base class. No functions implemented!")
    
    # def getFirstPoint(self):
    #     raise NotImplementedError("This is the abstract base class. No functions implemented!")

    # def getLastPoint(self):
    #     raise NotImplementedError("This is the abstract base class. No functions implemented!")

class LaneElementPoly3(LaneElement):
    def __init__(self, road_id,  x_poly, y_poly):
        self.road_id=road_id
        self.x_poly=x_poly
        self.y_poly=y_poly

        #The arc length is cached as it is computationally intensive. A call to getArcLength is lazy.
        self.arc_len=None
        self.angle_change=None
        self.angle_variance=None

        self.is_junction=False

    @staticmethod
    def fromOpenDrive(road_id, au,bu,cu,du,av,bv,cv,dv,hdg,x,y):
        #au+=x
        #av+=y
        u_poly=np.poly1d([du,cu,bu,au])
        v_poly=np.poly1d([dv,cv,bv,av])
        x_poly=u_poly*np.cos(hdg)-v_poly*np.sin(hdg)
        y_poly=u_poly*np.sin(hdg)+v_poly*np.cos(hdg)
        x_poly[0]+=x
        y_poly[0]+=y
        return LaneElementPoly3(road_id, x_poly, y_poly)

    @staticmethod
    def fromLITDLaneFormat(road_id, ax,bx,cx,dx,ay,by,cy,dy):
        x_poly=np.poly1d([dx,cx,bx,ax])
        y_poly=np.poly1d([dy,cy,by,ay])
        return LaneElementPoly3(road_id, x_poly, y_poly)

    def setAsJunctionElement(self):
        self.is_junction=True

    def calcPolyDerivate(self):
        x_der=np.poly1d(np.polyder(self.x_poly))
        y_der=np.poly1d(np.polyder(self.y_poly))
        return (x_der, y_der)

    def calcArcLength(self, start_p=0.0, end_p=1.0):
        deriv=self.calcPolyDerivate()
        x_der=deriv[0]
        y_der=deriv[1]
        res = scipy.integrate.quad(lambda p: np.sqrt(np.polyval(x_der,p)**2+np.polyval(y_der, p)**2), start_p, end_p)
        return res[0]

    def getArcLength(self):
        if(self.arc_len is None):
            self.arc_len=self.calcArcLength()
        return self.arc_len

    def getAngleChange(self):
        if(self.angle_change is None):
            poly_dx=np.poly1d(np.polyder(self.x_poly))
            poly_dy=np.poly1d(np.polyder(self.y_poly))          
            pnt_x_0=poly_dx(0.0)
            pnt_y_0=poly_dy(0.0)
            pnt_x_1=poly_dx(1.0)
            pnt_y_1=poly_dy(1.0)
            self.angle_change=np.arctan2(pnt_y_1,pnt_x_1)-np.arctan2(pnt_y_0, pnt_x_0)
            #print("angle_change befor warp is {}!".format(self.angle_change))
            x=np.fmod(self.angle_change, 2.0*np.pi)
            if(x>np.pi):
                x-=2.0*np.pi
            elif(x<-np.pi):
                x+=2.0*np.pi
            self.angle_change=x
        return self.angle_change


    def getAngleVariance(self, points=100):
        if(self.angle_variance is None):
            pspace=np.linspace(0.0, 1.0, points+1)
            pnts_x=self.x_poly(pspace)
            pnts_y=self.y_poly(pspace)
            pnts_dx=pnts_x[1:]-pnts_x[:-1]
            pnts_dy=pnts_y[1:]-pnts_y[:-1]
            #middle_change=self.getAngleChange()/(points-1)
            #print("Middle change: "+str(middle_change)+ " abs change: "+str(self.getAngleChange()))
            angle_variances=(np.arctan2(pnts_dy[1:],pnts_dx[1:])-np.arctan2(pnts_dy[:-1],pnts_dx[:-1]))**2
            self.angle_variance=np.sum(angle_variances)*points
        return self.angle_variance


    def getPixelPointList(self, pixel_per_meter=100, points=100):
        if(points<=0):
            print("getPixelPointList needs a points parameter which is > 0!")
            return []
        p=np.linspace(0.0, 1.0, points)
        x_array=pixel_per_meter*self.x_poly(p)
        y_array=pixel_per_meter*self.y_poly(p)
        return (x_array, y_array)
    
    
    #lane_id positive is left
    def addOffsetAndFit(self, lane_offsets, lane_id):
        if(lane_id==0):
            return False
        pspace=np.linspace(0.0, 1.0, 100)
        poly_dx=np.poly1d(np.polyder(self.x_poly))
        poly_dy=np.poly1d(np.polyder(self.y_poly))

        pnts_x=self.x_poly(pspace)
        pnts_y=self.y_poly(pspace)
        pnts_dx=poly_dx(pspace)
        pnts_dy=poly_dy(pspace)

        #opendrive lane ids are positive for left lanes and right for right lanes
        if(lane_id>0):
            angle=np.arctan2(pnts_dy,pnts_dx)+np.pi/2
        else:
            angle=np.arctan2(pnts_dy,pnts_dx)-np.pi/2

        pnts_nx=pnts_x+np.cos(angle)*lane_offsets[abs(lane_id)]
        pnts_ny=pnts_y+np.sin(angle)*lane_offsets[abs(lane_id)]

        if(lane_id>0):
            pspace=np.flip(pspace)
        poly_fx = np.poly1d(np.polyfit(pspace, pnts_nx, 3))
        poly_fy = np.poly1d(np.polyfit(pspace, pnts_ny, 3))

        self.x_poly=poly_fx
        self.y_poly=poly_fy
        
    def reverse(self):
        #Reverses the Polynomial parameter range from 0 to 1 -> 1 to 0
        d=self.x_poly[3]
        c=self.x_poly[2]
        b=self.x_poly[1]
        a=self.x_poly[0]
        self.x_poly=np.poly1d([-d, 3*d+c, -3*d-2*c-b, a+b+c+d])
        d=self.y_poly[3]
        c=self.y_poly[2]
        b=self.y_poly[1]
        a=self.y_poly[0]
        self.y_poly=np.poly1d([-d, 3*d+c, -3*d-2*c-b, a+b+c+d])        

    def __str__( self ):
        return "LaneElementPoly3: x = {0} + {1} * p + {2} * p² + {3} * p³ | y = {4} + {5} * p + {6} * p² + {7} * p³".format(self.x_poly[0],self.x_poly[1],self.x_poly[2],self.x_poly[3],self.y_poly[0],self.y_poly[1],self.y_poly[2],self.x_poly[3])
