from .LITD_RoadList import *

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
            if(RoadDecisions.MERGE in lane_suc):
                if(len(decision_list)>decision_id and decision_list[decision_id]==RoadDecisions.MERGE):
                    decision_id+=1
                result_list[2].append(RoadDecisions.MERGE)
                lane_id=lane_suc[RoadDecisions.MERGE]
            elif(RoadDecisions.NEXT in lane_suc):
                lane_id=lane_suc[RoadDecisions.NEXT]
                result_list[2].append(RoadDecisions.NEXT)
            else:
                print("ERROR: lane_suc has one entry bit is neither MERGE nor NEXT!")
                result_list[2].append(RoadDecisions.INVALID)
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
                    result_list[2].append(RoadDecisions.INVALID)
                    return result_list
            else:
                print("WARNING: lane_suc has no more decisions on this point!")
                result_list[2].append(RoadDecisions.INVALID)
                return result_list
        steps-=1
    return result_list
