from .road_list import *
from litdrive.selfdriving.enums import ManeuverState
import numpy as np

def getLaneListByDecisions(road_list:RoadList, start_id: int, decision_list, steps=10):
    #Returns a tuple with: a list of the lane objects, a list of ids, a list of the fired decisions (the last one is the decision that was used at this point (to detect merges etc.))
    result_list=(list(),list(),list())
    lane_id=start_id
    decision_id=0
    while(steps>0):
        print("Loop {}".format(steps))
        lane=road_list.lanes[lane_id]
        result_list[0].append(lane)
        result_list[1].append(lane_id)
        lane_suc=road_list.successors[lane_id]
        if(len(lane_suc)==0):
            #No more successors!
            return result_list
        if(len(lane_suc)==1):
            #We either have a NEXT or a MERGE.
            #we use the merge-decision if there is any. or just run over it, if there is none.
            if(ManeuverState.MERGE in lane_suc):
                if(len(decision_list)>decision_id and decision_list[decision_id]==ManeuverState.MERGE):
                    decision_id+=1
                result_list[2].append(ManeuverState.MERGE)
                lane_id=lane_suc[ManeuverState.MERGE]
            elif(ManeuverState.NEXT in lane_suc):
                lane_id=lane_suc[ManeuverState.NEXT]
                result_list[2].append(ManeuverState.NEXT)
            else:
                print("ERROR: lane_suc has one entry bit is neither MERGE nor NEXT!")
                result_list[2].append(ManeuverState.INVALID)
                return result_list
        else:
            if(len(decision_list)>decision_id):
                dec=decision_list[decision_id]
                if(dec in lane_suc):
                    lane_id=lane_suc[dec]
                    decision_id+=1
                    result_list[2].append(dec)
                else:
                    print("WARNING: lane_suc has no suitable successor for decision!")
                    result_list[2].append(ManeuverState.INVALID)
                    return result_list
            else:
                print("WARNING: lane_suc has no more decisions on this point!")
                result_list[2].append(ManeuverState.INVALID)
                return result_list
        steps-=1
    return result_list

def getLaneByPosition(road_list:RoadList, x:float, y:float, h:float, steps=1000):
    min_dist=1000000000000.0
    min_id=0
    pspace=np.linspace(0.0, 1.0, steps)
    for key,l in road_list.lanes.items():
        x_poly, y_poly = l.getPolys(False)
        dx_poly = x_poly.deriv()
        dy_poly = y_poly.deriv()
        pnts_x=x_poly(pspace)
        pnts_y=y_poly(pspace)
        pnts_x_deriv=dx_poly(pspace)
        pnts_y_deriv=dy_poly(pspace)
        pnts_dx=x-pnts_x
        pnts_dy=y-pnts_y
        angles=np.arctan2(pnts_y_deriv, pnts_x_deriv)
        pnts_dist=np.sqrt(pnts_dx**2+pnts_dy**2)
        for i in range(0, steps):
            angle=angles[i]-h
            while(angle>np.pi):
                angle-=2*np.pi
            while(angle<-np.pi):
                angle+=2*np.pi

            if(angle<np.pi/2.0 and angle>-np.pi/2.0 and pnts_dist[i]<min_dist):
                min_dist=pnts_dist[i]
                min_id=key
    return min_id
