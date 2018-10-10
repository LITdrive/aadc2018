from litdrive.roads.road_list import *
from litdrive.roads.road_access import *
from litdrive.selfdriving.enums import *


TRAJECTORY_ARRAY_SIZE = 10
TRAJECTORY_NUM_FIELDS = 12




class Planner:
    def __init__(self, road_file: str):

        self.state=PlannerState.NO_TRAJECTORIES
        self.error=PlannerErrors

        self.controller_current_id=0
        self.controller_current_p=0.0

        self.lanes_in_controller=list()
        self.ids_in_controller=list()
        self.decisions_in_controller = list()

        self.lanes_to_submit=list()
        self.ids_to_submit=list()
        self.decisions_to_submit=list()

        self.lanes_done=list()
        self.decisions_done=list()

        self.init_id=0

        self.rl = RoadList()

        if(not self.rl.loadFromFile(road_file)):
            raise Exception("Failed to open Pickle Road Map!")

        pass


    def initPlannerSettings(self, init_x:float, init_y:float):
        self.init_id=init_id

        return self.init_id

    #Management of the lanes in the controller


    def addManeuver(self, maneuver:ManeuverState):
        self.decisions_to_submit.append(maneuver)



    def update(self, controller_done_id, controller_current_id, controller_current_p):
        if(self.state<=PlannerState.INVALID ):
            print("ERROR: Planner is in INVALID-State!!!")
        elif(self.state==PlannerState.ERROR):
            print("ERROR: " + str(self.state))


        #Delete all elements up to the given ID
        last_id=0
        last_controller_id=0
        while len(self.ids_in_controller)>0 and self.ids_in_controller[0]<controller_done_id:
            print("planner: deleting controller lane id {}!".format(self.ids_in_controller[0]))
            self.lanes_in_controller.pop(0)
            self.ids_in_controller.pop(0)


        num_controller = len (self.ids_in_controller)
        if last_id==0:
            if(num_controller==0):
                print("Selected ids from Init")
                last_id=self.init_id
                last_controller_id=1
            else:
                print("Selected ids from lists")
                last_id=self.lanes_in_controller[-1]
                last_controller_id=self.ids_in_controller[-1]




        self.controller_current_id=controller_current_id
        self.controller_current_p =controller_current_p


        #num_controller = len (self.ids_in_controller)
        if(num_controller<TRAJECTORY_ARRAY_SIZE):
            if(len(self.decisions_in_controller)>0 and( self.decisions_in_controller[-1]==ManeuverState.LEFT or self.decisions_in_controller[-1]==ManeuverState.STRAIGHT or self.decisions_in_controller[-1]==ManeuverState.RIGHT or self.decisions_in_controller[-1]==ManeuverState.MERGE)):
                self.decisions_to_submit.insert(0,self.decisions_in_controller[-1])
            lanes, ids, dec = getLaneListByDecisions(self.rl, last_id, self.decisions_to_submit, TRAJECTORY_ARRAY_SIZE-num_controller+1)

            if(ManeuverState.INVALID in dec):
                print("WARNING: Decision has an INVALID state inside.")

            for d in dec:
                if(d==ManeuverState.LEFT or d==ManeuverState.STRAIGHT or d==ManeuverState.RIGHT or d==ManeuverState.MERGE):
                    if(d!=self.decisions_to_submit[0]):
                        print("WARNING: Used decision is not in to submit list!")
                    self.decisions_to_submit.pop(0)

            if(num_controller>0):
                self.lanes_in_controller.pop(num_controller-1)
                self.decisions_in_controller.pop(num_controller-1)
                self.ids_in_controller.pop(num_controller-1)
            self.lanes_in_controller.extend(ids)
            self.decisions_in_controller.extend(dec)
            controller_ids=list(range(last_controller_id, last_controller_id+len(ids)))
            print("New controller ids from {} to {}".format(last_controller_id, last_controller_id+len(ids)))
            self.ids_in_controller.extend(controller_ids)

            buffer = ([0] * TRAJECTORY_NUM_FIELDS * TRAJECTORY_ARRAY_SIZE)
            for i in range(0, len(ids)):
                x_poly, y_poly = lanes[i].getPolys(False)
                buffer[i*TRAJECTORY_NUM_FIELDS:(i+1)*TRAJECTORY_NUM_FIELDS]=[controller_ids[i], *tuple(x_poly)[::-1], *tuple(y_poly)[::-1], 0.0, 1.0, False]

            return buffer

        return None

    def getUpcommingLanes(self):
        return self.lanes_in_controller

    def getUpcommingDecisions(self):
        return self.decisions_in_controller




    def __str__(self):
        ret_str=list();
        ret_str.append("Planner-Object:\n")
        ret_str.append("Init-Lane-ID: ");
        ret_str.append(self.init_id.__str__());
        ret_str.append("\n")


        if(len(self.lanes_in_controller)>0):
            ret_str.append("Lanes in controller: ");
            ret_str.append(self.lanes_in_controller.__str__());
            ret_str.append("\n")

        if(len(self.ids_in_controller)>0):
            ret_str.append("IDS in controller: ");
            ret_str.append(self.ids_in_controller.__str__());
            ret_str.append("\n")

        if(len(self.decisions_in_controller)>0):
            ret_str.append("Decisions in controller: ");
            ret_str.append(self.decisions_in_controller.__str__());
            ret_str.append("\n")


        if(len(self.lanes_to_submit)>0):
            ret_str.append("Lanes to submit: ");
            ret_str.append(self.lanes_to_submit.__str__());
            ret_str.append("\n")

        if(len(self.ids_to_submit)>0):
            ret_str.append("IDS to submit: ");
            ret_str.append(self.ids_to_submit.__str__());
            ret_str.append("\n")

        if(len(self.decisions_to_submit)>0):
            ret_str.append("Decisions to submit: ");
            ret_str.append(self.decisions_to_submit.__str__());
            ret_str.append("\n")


        if(len(self.lanes_done)>0):
            ret_str.append("Lanes done: ");
            ret_str.append(self.lanes_done.__str__());
            ret_str.append("\n")

        if(len(self.decisions_done)>0):
            ret_str.append("desicions done: ");
            ret_str.append(self.decisions_done.__str__());
            ret_str.append("\n")

        return "".join(ret_str)





