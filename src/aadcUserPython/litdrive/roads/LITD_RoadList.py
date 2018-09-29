import numpy as np
import scipy.integrate
from enum import IntEnum


class RoadDecisions(IntEnum):
    NEXT=0
    LEFT=1
    STRAIGHT=2
    RIGHT=3
    PARK=4
    MAX=5


class RoadList:
    def __init__(self):
        self.lanes=dict()
        self.roads=dict()
        self.decisions_start=dict()
        self.decisions_end=dict()
        return

    def addLaneElement(self, element, id, decision_start, decision_end):
        if(not issubclass(type(element), LaneElement)):
            raise Exception("Element is not a Object with Base-class LaneElement!")        
        self.lanes[id]=element
        self.decisions_start[id]=decision_start
        self.decisions_end[id]=decision_end

    def removeLaneElement(self, id):
        if(id in self.lanes):
            retval=self.lanes[id]
            self.lanes[id]=None
            return retval
        return None
    
    def addRoad(self, element, road_id):      
        self.roads[road_id]=element

    
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
    def __init__(self, x_poly, y_poly):
        self.x_poly=x_poly
        self.y_poly=y_poly

        #The arc length is cached as it is computationally intensive. A call to getArcLength is lazy.
        self.arc_len=-1.0

    @staticmethod
    def fromOpenDrive(au,bu,cu,du,av,bv,cv,dv,hdg,x,y):
        #au+=x
        #av+=y
        u_poly=np.poly1d([du,cu,bu,au])
        v_poly=np.poly1d([dv,cv,bv,av])
        x_poly=u_poly*np.cos(hdg)-v_poly*np.sin(hdg)
        y_poly=u_poly*np.sin(hdg)+v_poly*np.cos(hdg)
        x_poly[0]+=x
        y_poly[0]+=y
        return LaneElementPoly3(x_poly, y_poly)

    @staticmethod
    def fromLITDLaneFormat(ax,bx,cx,dx,ay,by,cy,dy):
        x_poly=np.poly1d([dx,cx,bx,ax])
        y_poly=np.poly1d([dy,cy,by,ay])
        return LaneElementPoly3(x_poly, y_poly)


    def calcPolyDerivate(self):
        poly_dim=len(self.x_poly)
        mult=np.arange(poly_dim,-1,-1)
        x_der=np.poly1d((self.x_poly.c*mult)[0:poly_dim])
        y_der=np.poly1d((self.y_poly.c*mult)[0:poly_dim])
        return (x_der, y_der)

    def calcArcLength(self):
        deriv=self.calcPolyDerivate()
        x_der=deriv[0]
        y_der=deriv[1]
        res = scipy.integrate.quad(lambda p: np.sqrt(np.polyval(x_der,p)**2+np.polyval(y_der, p)**2), 0, 1)
        return res[0]

    def getArcLength(self):
        if(self.arc_len<0):
            self.arc_len=self.calcArcLength()
        return self.arc_len

    def getPixelPointList(self, points=100):
        if(points<=0):
            print("getPixelPointList needs a points parameter which is > 0!")
            return []
        p=np.linspace(0.0, 1.0, points)
        x_array=self.x_poly(p)
        y_array=self.y_poly(p)
        return (x_array, y_array)
    
    
    #road_offset positive is right.
    def addOffsetAndFit(self, road_offset, is_left):
        pspace=np.linspace(0.0, 1.0, 100)
        poly_dx=np.poly1d(np.polyder(self.x_poly))
        poly_dy=np.poly1d(np.polyder(self.y_poly))

        pnts_x=self.x_poly(pspace)
        pnts_y=self.y_poly(pspace)
        pnts_dx=poly_dx(pspace)
        pnts_dy=poly_dy(pspace)

        if(is_left):
            angle=np.arctan2(pnts_dy,pnts_dx)+np.pi/2
        else:
            angle=np.arctan2(pnts_dy,pnts_dx)-np.pi/2

        pnts_nx=pnts_x+np.cos(angle)*road_offset
        pnts_ny=pnts_y+np.sin(angle)*road_offset

        if(is_left):
            pspace=np.flip(pspace)
        poly_fx = np.poly1d(np.polyfit(pspace, pnts_nx, 3))
        poly_fy = np.poly1d(np.polyfit(pspace, pnts_ny, 3))

        self.x_poly=poly_fx
        self.y_poly=poly_fy
        


    def __str__( self ):
        return "LaneElementPoly3: x = {0} + {1} * p + {2} * p² + {3} * p³ | y = {4} + {5} * p + {6} * p² + {7} * p³".format(self.x_poly[3],self.x_poly[2],self.x_poly[1],self.x_poly[0],self.y_poly[3],self.y_poly[2],self.y_poly[1],self.x_poly[0])





class LITD_LaneList:
    def __init__(self):
        #Holds a list of elements with the common superclass LITD_LaneElement
        self.lanes=[]
        self.maneuvers=[]
        self.roads=[]


if __name__ == '__main__':
    p3=LaneElementPoly3.fromLITDLaneFormat(1,0,2,0,1,0,0,0)
    print(p3)
    print(p3.calcArcLength())