from .road_list import *
import numpy as np
from litdrive.selfdriving.enums import ManeuverState


def fitPolysToPoly(list_p1d_u, list_p1d_v, list_hdg, list_x, list_y):
    pspace=np.linspace(0.0, 1.0, 100)
    pnts_x=list()
    pnts_y=list()
    if(len(list_p1d_v)!=len(list_p1d_u) or len(list_p1d_v)!=len(list_hdg) or len(list_p1d_v)!=len(list_x) or len(list_p1d_v)!=len(list_y)):
        raise Exception("ERROR: Parameter lists must have equal lengths!")
    for i in range(0,len(list_p1d_v)):
        p1d_u=np.poly1d(list_p1d_u[i])
        p1d_v=np.poly1d(list_p1d_v[i])
        p1d_x=p1d_u*np.cos(list_hdg[i])-p1d_v*np.sin(list_hdg[i])
        p1d_y=p1d_u*np.sin(list_hdg[i])+p1d_v*np.cos(list_hdg[i])
        p1d_x[0]+=list_x[i]
        p1d_y[0]+=list_y[i]
        pnts_x.append(p1d_x(pspace))
        pnts_y.append(p1d_y(pspace))
    pnts_x=np.concatenate(pnts_x)
    pnts_y=np.concatenate(pnts_y)
    pspace=np.linspace(0.0, 1.0, 100*len(list_p1d_u))
    p1d_fx = np.poly1d(np.polyfit(pspace, pnts_x, 3))
    p1d_fy = np.poly1d(np.polyfit(pspace, pnts_y, 3))

    return (p1d_fx[0],p1d_fx[1],p1d_fx[2],p1d_fx[3],p1d_fy[0],p1d_fy[1],p1d_fy[2],p1d_fy[3])

def getPoly3Params(geo, scaling_factor, road_id):
    try:
        geo_x=float(geo.get("x"))*scaling_factor
        geo_y=float(geo.get("y"))*scaling_factor
    except (TypeError, ValueError) as e:
        raise Exception("Road {} Geometry has no or invalid x or y coordinates!".format(road_id))
        
    # Try to get a paramPoly3
    geo_pp3 = geo.findall("paramPoly3")
    if(geo_pp3 is None or len(geo_pp3)>1):
        raise Exception("Road {} Geometry has no or multiple paramPoly3. Only one paramPoly3 per road is supported!".format(road_id))
    
    geo_pp3=geo_pp3[0]
    
    try:
        geo_hdg = float(geo.get("hdg"))
        geo_length = float(geo.get("length"))
    except (TypeError, ValueError) as e:
        raise Exception("Road {} Geometry has an invalid hdg or length entry!".format(road_id))
    
    try:
        pp3_aU = float(geo_pp3.get("aU"))*scaling_factor
        pp3_bU = float(geo_pp3.get("bU"))*scaling_factor
        pp3_cU = float(geo_pp3.get("cU"))*scaling_factor
        pp3_dU = float(geo_pp3.get("dU"))*scaling_factor
        pp3_aV = float(geo_pp3.get("aV"))*scaling_factor
        pp3_bV = float(geo_pp3.get("bV"))*scaling_factor
        pp3_cV = float(geo_pp3.get("cV"))*scaling_factor
        pp3_dV = float(geo_pp3.get("dV"))*scaling_factor
    except (TypeError, ValueError) as e:
        raise Exception("Road {} paramPoly3 has invalid {a,b,c,d}{U,V} entries!".format(road_id))
    return (pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y)

def ReadOpenDrive(xml_tree, scaling_factor=1, lane_offsets=(0.0, 0.25, 0.75), angle_left_threshold=np.pi/4, angle_right_threshold=-np.pi/4):
    road_list = RoadList()
    new_lane_id=1
    #new_road_id=0
    new_junction_id=0

    if(xml_tree is None):
        raise Exception("OpenDRIVE not loaded, exiting...")

    root_node=xml_tree.getroot()


    if(root_node is None):
        raise Exception("ERROR: OpenDRIVE has no root-node, quitting!")

    header = root_node.find("header")

    if(header is not None):
        out_str=list()
        out_str.append("OpenDRIVE File Header Information:\n")
        if header.get('revMajor') is not None:
            out_str.append("Major Revision: ")
            out_str.append(str(header.get('revMajor')))
            out_str.append("\n")

        if header.get('revMinor') is not None:
            out_str.append("Minor Revision: ")
            out_str.append(str(header.get('revMinor')))
            out_str.append("\n")

        if header.get('name') is not None:
            out_str.append("Name: ")
            out_str.append(str(header.get('name')))
            out_str.append("\n")

        if header.get('version') is not None:
            out_str.append("Version: ")
            out_str.append(str(header.get('version')))
            out_str.append("\n")

        if header.get('date') is not None:
            out_str.append("Date: ")
            out_str.append(str(header.get('revMajor')))
            out_str.append("\n")

        if header.get('north') is not None:
            out_str.append("North: ")
            out_str.append(str(header.get('north')))
            out_str.append("\n")

        if header.get('south') is not None:
            out_str.append("South: ")
            out_str.append(str(header.get('south')))
            out_str.append("\n")

        if header.get('east') is not None:
            out_str.append("East: ")
            out_str.append(str(header.get('east')))
            out_str.append("\n")

        if header.get('west') is not None:
            out_str.append("West: ")
            out_str.append(str(header.get('west')))
            out_str.append("\n")

        if header.get('vendor') is not None:
            out_str.append("Vendor: ")
            out_str.append(str(header.get('vendor')))
            out_str.append("\n")

        print("".join(out_str))
    else:
        print("WARNING: OpenDRIVE file has no header tag!")

        
    print("INFO: Loading junctions...")
    #Dictionary with the junction-id as index
    #each entry then holds another dict with the connection id as index.
    #the third dict then holds all values.
    #Clarification: incomingRoad is the Road ID that comes to the junction, connectingRoad is the Road ID inside the junction.
    #If the contactPoint is end, then the Polynomial of the connectingRoad is reversed.
    junction_dict=dict()
        
    #First, iterate over all junctions and store them for later usage.
    junctions = root_node.findall("junction")
    if(junctions is None):
        print("WARNING: OpenDRIVE has no junctions!")

    for junction in junctions:
        try:
            junction_id=int(junction.get("id"))
        except (ValueError, TypeError) as e:
            raise Exception("ERROR: Junction with invalid ID")

        connections=junction.findall("connection")
        if(connections is None):
            raise Exception("ERROR: Junction {} has no connection entries!")
            
        junction_dict[junction_id]=dict()
            
        for connection in connections:
            try:
                connecting_road=int(connection.get("connectingRoad"))
                incoming_road=int(connection.get("incomingRoad"))
                connection_id=int(connection.get("id"))
            except (ValueError, TypeError) as e:
                raise Exception("ERROR: Junction {} has a connection with invalid connectingRoad, incomingRoad or id!".format(junction_id))
            
            contact_point = connection.get("contactPoint")
            if(contact_point is None):
                raise Exception("ERROR: Junction {} has no contactPoint!".format(junction_id))
                
            lane_links=connection.findall("laneLink")
            if(lane_links is None or len(lane_links)!=1):
                raise Exception("ERROR: Junction {} connection {} has no or multiple laneLink tags, only one supported!".format(junction_id, connection_id))
            lane_link=lane_links[0]
            
            try:
                link_from=int(lane_link.get("from"))
                link_to=int(lane_link.get("to"))
            except (ValueError, TypeError) as e:
                raise Exception("ERROR: Junction {} connection {} laneLink has invalid from or to tags!".format(junction_id, connection_id))
            
            junction_dict[junction_id][connection_id]={"connectingRoad":connecting_road, "incomingRoad": incoming_road, "from":link_from, "to": link_to, "contactPoint": contact_point}


    # roads is a list of "road" tags
    print("INFO: Loading roads...")

    roads = root_node.findall("road") 

    lane_pre_od_ids=dict()
    lane_suc_od_ids=dict()

    if(roads is None):
        raise Exception("ERROR: OpenDRIVE has no roads!")
    else:
        print("INFO: OpenDRIVE has {} roads!".format(len(roads)))
        overall_len=0.0
        overall_lanes=0
        #This holds all junction lane id's with the successor as first index and predecessor as second index. inside there is a list of roads. This is used to identify what is left and what is right
        junction_lane_table=dict()
        for road in roads:
            
            # Get the Road-ID and the id of the junction it belongs to
            try:
                road_id=int(road.get("id"))
            except (TypeError, ValueError) as e:
                raise Exception("ERROR: Found a Road with no valid ID, quitting!")

            try:
                road_junction = int(road.get("junction"))
            except (TypeError, ValueError) as e:
                raise Exception("ERROR: Road {} Junction entry is invalid!".format(road_id))                    
            
            try:
                road_len=float(road.get("length"))*scaling_factor
                overall_len+=road_len
            except:
                print("WARNING: Road {} has no valid length, ignoring!".format(road_id))
            plan_view=road.find("planView")
            if(plan_view is None):
                raise Exception("Road {} has no plan view!".format(road_id))
                
            geometries = plan_view.findall("geometry")
            if(geometries is None):
                raise Exception("ERROR: Road {} PlanView has no Geometry entries!".format(road_id))
            if(len(geometries)!=1):
                print("WARNING: Road {} PlanView has multiple Geometry entries! Fitting poly".format(road_id))
                #raise Exception("ERROR: Road {} PlanView has no or multiple Geometry entries!".format(road_id))
                list_p1d_u=list()
                list_p1d_v=list()
                list_hdg=list()
                list_x=list()
                list_y=list()
                for geo in geometries:

                    poly_params=getPoly3Params(geo, scaling_factor, road_id)

                    #list_p1d_u.append((pp3_dU,pp3_cU,pp3_bU,pp3_aU))
                    list_p1d_u.append(poly_params[3::-1])
                    #same with V
                    list_p1d_v.append(poly_params[7:3:-1])
                    list_hdg.append(poly_params[8])
                    list_x.append(poly_params[9])
                    list_y.append(poly_params[10])
                pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV = fitPolysToPoly(list_p1d_u, list_p1d_v, list_hdg, list_x, list_y)
                geo_hdg=0
                geo_x=0
                geo_y=0
            else:
                geo=geometries[0]

                pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y = getPoly3Params(geo, scaling_factor, road_id)
                        
            if(road_junction>=0):
                #To have the right "polynomial direction" of a junction element, we look up the "contactPoint" in the junction for this road
                #it this value is "end" we reverse the Polynom to deliver the function values from 1 to 0, so we have the right direction.

                #print("INFO: Road {} is a junction road and becomes lane {}".format(road_id, new_lane_id))

                if(road_junction not in junction_dict):
                    raise Exception("Road {} uses junction id {} which does not exist!".format(road_id, road_junction))
                junction_active=junction_dict[road_junction].items()
                lep3=LaneElementPoly3.fromOpenDrive(road_id, pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y)
                lep3.setAsJunctionElement()
                for key, val in junction_active:
                    if("connectingRoad" not in val or "contactPoint" not in val):
                        raise Exception("WARNING: junction_dict is malformed, entry Junction ID {} connection id {} has no \"contactPoint\" or \"connectingRoad\"!".format(road_junction, key))
                    if(val["connectingRoad"]==road_id and val["contactPoint"]=="end"):
                        #print("INFO: contactPoint for Junction ID {} connection id {} is end, reversing road id {}".format(road_junction, key, road_id))
                        lep3.reverse()
                        break
                lanes = road.find("lanes")
                if(lanes is None):
                    raise Exception("ERROR: Road {} has no lanes tag!".format(road_id))
                # for laneOffset in lanes.findall("laneOffset"):
                #     sPos = float(laneOffset.get("s"))
                #     a = float(laneOffset.get("a"))
                #     b = float(laneOffset.get("b"))
                #     c = float(laneOffset.get("c"))
                #     d = float(laneOffset.get("d"))
                #     if(abs(a)>0.001 or abs(b)>0.001 or abs(c)>0.001 or abs(d)>0.001):
                #         print("WARNING: Road {} has an laneOffset that could be important! Ignoring!".format(road_id))

                rl = RoadElement()

                lane_sections=lanes.findall("laneSection")
                if(lane_sections is None or len(lane_sections)>1):
                    raise Exception("WARNING: Road {} has no or multiple lane sections!".format(road_id))
                
                #could be replaced by a for-each loop, if required...
                lane_section=lane_sections[0]

                section_right=lane_section.find("right")
                section_left=lane_section.find("left")
                if(section_left is not None and section_right is not None):
                    raise Exception("ERROR: Road {} is a junction and has left and right lanes! Unsupported!".format(road_id))
                elif(section_right is not None):
                    if(len(section_right.findall("lane"))>1):
                        print("WARNING: Road {} right has multiple lanes but is a crossing, ignoring!".format(road_id))
                    # Add this lane as the inner right lane
                    rl.addLane(new_lane_id, -1)
                    right_lane=section_right.find("lane")
                    right_link=right_lane.find("link")
                    if(right_link is None):
                        print("WARNING: Junction Road {} right lane has no link!".format(road_id))
                    else:
                        right_suc=right_link.find("successor")
                        if(right_suc is not None):
                            try:
                                lane_suc=int(right_suc.get("id"))
                            except (ValueError, TypeError) as e:
                                raise Exception("ERROR: Road {} link successor has an invalid id!".format(road_id))
                        right_pre=right_link.find("predecessor")
                        if(right_pre is not None):
                            try:
                                lane_pre=int(right_pre.get("id"))
                            except (ValueError, TypeError) as e:
                                raise Exception("ERROR: Road {} link predecessor has an invalid id!".format(road_id))                                

                elif(section_left is not None):
                    if(len(section_left.findall("lane"))>1):
                        print("WARNING: Road {} left has multiple lanes but is a crossing, ignoring!".format(road_id))
                    # Add this lane as the inner left lane
                    rl.addLane(new_lane_id, 1)
                    left_lane=section_left.find("lane")
                    left_link=left_lane.find("link")
                    if(left_link is None):
                        print("WARNING: Junction Road {} left lane has no link!".format(road_id))
                    else:
                        left_suc=left_link.find("successor")
                        if(left_suc is not None):
                            try:
                                lane_suc=int(left_suc.get("id"))
                            except (ValueError, TypeError) as e:
                                raise Exception("ERROR: Road {} link successor has an invalid id!".format(road_id))
                        left_pre=left_link.find("predecessor")
                        if(left_pre is not None):
                            try:
                                lane_pre=int(left_pre.get("id"))
                            except (ValueError, TypeError) as e:
                                raise Exception("ERROR: Road {} link predecessor has an invalid id!".format(road_id)) 
                else:
                    raise Exception("ERROR: Road {} has no left or right, but one is required for this junction!".format(road_id))
                road_list.addLaneElement(lep3, new_lane_id, None, None)
                lane_pre_od_ids[new_lane_id]=lane_pre
                lane_suc_od_ids[new_lane_id]=lane_suc
                road_list.addRoad(rl, road_id)
                new_lane_id+=1
                road_link=road
            else:
                lanes = road.find("lanes")
                if(lanes is None):
                    raise Exception("ERROR: Road {} has no lanes tag!".format(road_id))
                for laneOffset in lanes.findall("laneOffset"):
                    sPos = float(laneOffset.get("s"))
                    a = float(laneOffset.get("a"))
                    b = float(laneOffset.get("b"))
                    c = float(laneOffset.get("c"))
                    d = float(laneOffset.get("d"))
                    if(abs(a)>0.001 or abs(b)>0.001 or abs(c)>0.001 or abs(d)>0.001):
                        print("WARNING: Road {} has an laneOffset that could be important! Ignoring!".format(road_id))
                    
                lane_sections=lanes.findall("laneSection")
                if(lane_sections is None or len(lane_sections)>1):
                    raise Exception("WARNING: Road {} has no multiple lane sections!".format(road_id))
                
                #could be replaced by a for-each loop, if required...
                lane_section=lane_sections[0]

                rl = RoadElement()
                
                section_left=lane_section.find("left")
                if(section_left is not None):
                    lanes_left=section_left.findall("lane")
                    if(lanes_left is None):
                        raise Exception("ERROR: Road {} left has no lanes!".format(road_id))
                    for lane_left in lanes_left:
                        try:
                            left_id=int(lane_left.get("id"))
                        except (ValueError, TypeError) as e:
                            raise Exception("ERROR: Road {} has a left entry with invalid id!".format(road_id))
                        if(not rl.addLane(new_lane_id, left_id)):
                            raise Exception("ERROR: Road {} left id {} is not supported!".format(road_id, left_id))
                        lep3=LaneElementPoly3.fromOpenDrive(road_id, pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y)
                        lep3.addOffsetAndFit(lane_offsets, left_id)
                        lane_pre=None
                        lane_suc=None
                        left_link=lane_left.find("link")
                        if(left_link is None):
                            print("WARNING: Road {} left lane id {} has no link entry!".format(road_id, left_id))
                        else:
                            left_pre=left_link.find("predecessor")
                            if(left_pre is not None):
                                try:
                                    lane_pre=int(left_pre.get("id"))
                                except (ValueError, TypeError) as e:
                                    raise Exception("ERROR: Road {} left lane id {} predecessor has an invalid id!".format(road_id, left_id))
                            left_suc=left_link.find("successor")
                            if(left_suc is not None):
                                try:
                                    lane_suc=int(left_suc.get("id"))
                                except (ValueError, TypeError) as e:
                                    raise Exception("ERROR: Road {} left lane id {} successor has an invalid id!".format(road_id, left_id))
                        road_list.addLaneElement(lep3, new_lane_id, None, None)
                        lane_pre_od_ids[new_lane_id]=lane_pre
                        lane_suc_od_ids[new_lane_id]=lane_suc
                        new_lane_id+=1
                else:
                    print("WARNING: Road {} has a laneSection without a left lane!".format(road_id))
                    
                section_right=lane_section.find("right")
                if(section_right is not None):
                    lanes_right=section_right.findall("lane")
                    if(lanes_right is None):
                        raise Exception("ERROR: in Road {} right has no lanes!".format(road_id))
                    for lane_right in lanes_right:
                        try:
                            right_id=int(lane_right.get("id"))
                        except (ValueError, TypeError) as e:
                            raise Exception("ERROR: Road {} has a right entry with invalid id!".format(road_id))
                        if(not rl.addLane(new_lane_id, right_id)):
                            raise Exception("ERROR: Road {} right id {} is not supported!".format(road_id, right_id))                            
                        lep3=LaneElementPoly3.fromOpenDrive(road_id, pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y)
                        lep3.addOffsetAndFit(lane_offsets, right_id)
                        lane_pre=None
                        lane_suc=None
                        right_link=lane_right.find("link")
                        if(right_link is None):
                            print("WARNING: Road {} right lane id {} has no link entry!".format(road_id, right_id))
                        else:
                            right_pre=right_link.find("predecessor")
                            if(right_pre is not None):
                                try:
                                    lane_pre=int(right_pre.get("id"))
                                except (ValueError, TypeError) as e:
                                    raise Exception("ERROR: Road {} right lane id {} predecessor has an invalid id!".format(road_id, right_id))
                            right_suc=right_link.find("successor")
                            if(right_suc is not None):
                                try:
                                    lane_suc=int(right_suc.get("id"))
                                except (ValueError, TypeError) as e:
                                    raise Exception("ERROR: Road {} right lane id {} successor has an invalid id!".format(road_id, right_id))
                        road_list.addLaneElement(lep3, new_lane_id, None, None)
                        lane_pre_od_ids[new_lane_id]=lane_pre
                        lane_suc_od_ids[new_lane_id]=lane_suc
                        new_lane_id+=1
                else:
                    print("WARNING: Road {} has a laneSection without a right lane!".format(road_id))

                road_list.addRoad(rl, road_id)
        for key,lane in road_list.lanes.items():
            print("Road: {} lane {}". format(lane.road_id, key))
        print(road_list.successors)
        print(road_list.predecessors)

        print("INFO: Creating successor lists...")
        #Loop all roads a second time to generate the successor list.

        for road in roads:
            
            # Get the Road-ID and the id of the junction it belongs to
            try:
                road_id=int(road.get("id"))
            except (TypeError, ValueError) as e:
                raise Exception("ERROR: Found a Road with no valid ID, quitting!")

            try:
                road_junction = int(road.get("junction"))
            except (TypeError, ValueError) as e:
                raise Exception("ERROR: Road {} Junction entry is invalid!".format(road_id))
                
            #get the link-tag. none or one exsits
            road_link=road.find("link")
            if(road_link is None):
                print("WARNING: Road {} has no link-tag!".format(road_id))
            else:
                #process the sucessor tag
                road_link_suc=road_link.find("successor")
                if(road_link_suc is None):
                    print("WARNING: Road {} link has no successor-tag!".format(road_id))
                else:
                    try:
                        suc_id=int(road_link_suc.get("elementId"))
                    except (ValueError, TypeError) as e:
                        raise Exception("ERROR: Road {} succecessor has no or invalid elementId".format(road_id))
                    suc_type=road_link_suc.get("elementType")
                    suc_contact_point=road_link_suc.get("contactPoint")
                    if(suc_type is None or suc_contact_point is None):
                        raise Exception("ERROR: Road {} succecessor has no elementType or contactPoint".format(road_id))
                    
                    if(suc_type.lower()=="road"):
                        #if the predecessor is a road, the next id can just be used.
                        if(suc_id not in road_list.roads):
                            raise Exception("ERROR: Road {} successor road {} does not exist!".format(road_id, suc_id))
                        
                        road=road_list.roads[road_id]
                        suc_road=road_list.roads[suc_id]
                        #do the right lanes 
                        for i in [-2,-1]:
                            lane_id=road.getLaneByOpenDriveID(i)
                            if(lane_id is not None):
                                od_lane_id=lane_suc_od_ids[lane_id]
                                if(od_lane_id is None):
                                    print("WARNING: Road {} has no successor!")
                                    road_list.setLaneSuccessorDict(lane_id, dict())
                                else:
                                    suc_lane_id=suc_road.getLaneByOpenDriveID(od_lane_id)
                                    if(suc_lane_id is None):
                                        raise Exception("ERROR: everythings fucked up here. I hate opendrive! lane_id {}, od_lane_id {}".format(lane_id,od_lane_id))
                                    road_list.setLaneSuccessorDict(lane_id,{ManeuverState.NEXT: suc_lane_id})

                        for i in [1,2]:
                            lane_id=road.getLaneByOpenDriveID(i)
                            if(lane_id is not None):
                                od_lane_id=lane_suc_od_ids[lane_id]
                                if(od_lane_id is None):
                                    print("WARNING: Road {} has no successor!")
                                    road_list.setLanePredecessorDict(lane_id, dict())
                                else:
                                    suc_lane_id=suc_road.getLaneByOpenDriveID(od_lane_id)
                                    if(suc_lane_id is None):
                                        raise Exception("ERROR: everythings fucked up here. I hate opendrive! lane_id {}, od_lane_id {}".format(lane_id,od_lane_id))
                                    road_list.setLanePredecessorDict(lane_id,{ManeuverState.NEXT: suc_lane_id})

                    elif(suc_type.lower()=="junction"):
                        #construct a decision
                        #print("INFO: Road {} has junction as successor!".format(road_id))

                        #first calculate the successor for the right incomingRoad
                        junction_roads_list=list()
                        if(suc_id not in junction_dict or junction_dict[suc_id] is None):
                            raise Exception("ERROR Road {} uses unkown junction {} as successor!".format(road_id, suc_id))
                        junction_active=junction_dict[suc_id].items()
                        for index, connection_active in junction_active:
                            if(connection_active["incomingRoad"]==road_id):
                                print("Adding road id {} to successor list!".format(connection_active["connectingRoad"]))
                                junction_roads_list.append(connection_active["connectingRoad"])
                        
                        #the successors are junction-roads which only have one lane. Easy to identify.
                        suc_roads_dict=dict()
                        if(len(junction_roads_list)==1):
                            suc_road_id=junction_roads_list[0]
                            suc_lane_id=road_list.roads[suc_road_id].getLaneByOpenDriveID(-1)
                            if(suc_lane_id is None):
                                suc_lane_id=road_list.roads[suc_road_id].getLaneByOpenDriveID(1)
                                if(suc_lane_id is None):
                                    raise Exception("ERROR: No lane id for road {} junction successor road {} found!".format(road_id, suc_road_id))
                            suc_lane=road_list.lanes[suc_lane_id]
                            suc_roads_dict[ManeuverState.MERGE]=suc_lane_id
                            lane_id=road_list.roads[road_id].getLaneByOpenDriveID(-1)
                            if(lane_id is None):
                                raise Exception("ERROR: road {} has junction {} as successor, but no right lane!".format(road_id, suc_id))
                            road_list.setLaneSuccessorDict(lane_id, suc_roads_dict)                                
                        elif(len(junction_roads_list)>1):
                            for suc_road_id in junction_roads_list:
                                #Junction-Roads do only have one lane, so we do not need to lookup them but just guess.
                                suc_lane_id=road_list.roads[suc_road_id].getLaneByOpenDriveID(-1)
                                if(suc_lane_id is None):
                                    suc_lane_id=road_list.roads[suc_road_id].getLaneByOpenDriveID(1)
                                    if(suc_lane_id is None):
                                        raise Exception("ERROR: No lane id for road {} junction successor road {} found!".format(road_id, suc_road_id))
                                suc_lane=road_list.lanes[suc_lane_id]
                                if(suc_lane.getAngleChange()>angle_left_threshold):
                                    if(ManeuverState.LEFT in suc_roads_dict):
                                        raise Exception("ERROR: road {} junction successor {} road has already a LEFT entry! {}".format(road_id, suc_id, suc_roads_dict))
                                    suc_roads_dict[ManeuverState.LEFT]=suc_lane_id
                                elif(suc_lane.getAngleChange()<angle_right_threshold):
                                    if(ManeuverState.RIGHT in suc_roads_dict):
                                        raise Exception("ERROR: road {} junction successor {} road has already a RIGHT entry! {}".format(road_id, suc_id, suc_roads_dict))
                                    suc_roads_dict[ManeuverState.RIGHT]=suc_lane_id                                        
                                else:
                                    if(ManeuverState.STRAIGHT in suc_roads_dict):
                                        raise Exception("ERROR: road {} junction successor {} road has already a STRAIGHT entry! {}".format(road_id, suc_id, suc_roads_dict))
                                    suc_roads_dict[ManeuverState.STRAIGHT]=suc_lane_id
                        else:
                            raise Exception("ERROR Road {} junction {} successor has no valid roads!".format(road_id, suc_id))
                        #The Predecessor entry for a junction is always for the right lane from opendrive.
                        lane_id=road_list.roads[road_id].getLaneByOpenDriveID(-1)
                        if(lane_id is None):
                            raise Exception("ERROR: road {} has junction {} as successor, but no right lane!".format(road_id, suc_id))
                        road_list.setLaneSuccessorDict(lane_id, suc_roads_dict)
                    else:
                        raise Exception("ERROR: Road {} prececessor elementType is invalid!".format(road_id))                            

        #print("INFO: Creating predecessor lists...")
        #Loop all roads a third time to generate the predecessor list.
        pre_roads_list=list()           
        for road in roads:
            
            # Get the Road-ID and the id of the junction it belongs to
            try:
                road_id=int(road.get("id"))
            except (TypeError, ValueError) as e:
                raise Exception("ERROR: Found a Road with no valid ID, quitting!")

            try:
                road_junction = int(road.get("junction"))
            except (TypeError, ValueError) as e:
                raise Exception("ERROR: Road {} Junction entry is invalid!".format(road_id))
                
            #get the link-tag. none or one exsits
            road_link=road.find("link")
            if(road_link is None):
                print("WARNING: Road {} has no link-tag!".format(road_id))
            else:
                #Process the predecessor tag
                road_link_pre=road_link.find("predecessor")
                if(road_link_pre is None):
                    print("WARNING: Road {} link as no predecessor-tag!".format(road_id))
                else:
                    try:
                        pre_id=int(road_link_pre.get("elementId"))
                    except (ValueError, TypeError) as e:
                        raise Exception("ERROR: Road {} predecessor has no or invalid elementId".format(road_id))
                    pre_type=road_link_pre.get("elementType")
                    pre_contact_point=road_link_pre.get("contactPoint")
                    if(pre_type is None or pre_contact_point is None):
                        raise Exception("ERROR: Road {} predecessor has no elementType or contactPoint".format(road_id))
                    
                    if(pre_type.lower()=="road"):
                        if(pre_id not in road_list.roads):
                            raise Exception("ERROR: Road {} successor road {} does not exist!".format(road_id, pre_id))
                        
                        road=road_list.roads[road_id]
                        suc_road=road_list.roads[pre_id]
                        #do the right lanes 
                        for i in [-2,-1]:
                            lane_id=road.getLaneByOpenDriveID(i)
                            if(lane_id is not None):
                                od_lane_id=lane_pre_od_ids[lane_id]
                                if(od_lane_id is None):
                                    print("WARNING: Road {} has no successor!")
                                    road_list.setLanePredecessorDict(lane_id, dict())
                                else:
                                    suc_lane_id=suc_road.getLaneByOpenDriveID(od_lane_id)
                                    if(suc_lane_id is None):
                                        raise Exception("ERROR: everythings fucked up here. I hate opendrive! lane_id {}, od_lane_id {}".format(lane_id,od_lane_id))
                                    road_list.setLanePredecessorDict(lane_id,{ManeuverState.NEXT: suc_lane_id})

                        for i in [1,2]:
                            lane_id=road.getLaneByOpenDriveID(i)
                            if(lane_id is not None):
                                od_lane_id=lane_pre_od_ids[lane_id]
                                if(od_lane_id is None):
                                    print("WARNING: Road {} lane id {} has no successor!".format(road_id, lane_id))
                                    road_list.setLaneSuccessorDict(lane_id, dict())
                                else:
                                    suc_lane_id=suc_road.getLaneByOpenDriveID(od_lane_id)
                                    if(suc_lane_id is None):
                                        raise Exception("ERROR: everythings fucked up here. I hate opendrive! lane_id {}, od_lane_id {}".format(lane_id,od_lane_id))
                                    print("Found left predecessor, road {}, lane {}, od_lane {}, suc_lane {}, pre_id {}".format(road_id, lane_id, od_lane_id, suc_lane_id, pre_id))
                                    road_list.setLaneSuccessorDict(lane_id,{ManeuverState.NEXT: suc_lane_id})

                    elif(pre_type.lower()=="junction"):
                        #construct a decision
                        print("INFO: Road {} has junction as predecessor!".format(road_id))
                        junction_roads_list=list()
                        if(pre_id not in junction_dict or junction_dict[pre_id] is None):
                            raise Exception("ERROR Road {} uses unkown junction {} as predecessor!".format(road_id, pre_id))
                        junction_active=junction_dict[pre_id].items()
                        for index, connection_active in junction_active:
                            if(connection_active["incomingRoad"]==road_id):
                                print("Adding road id {} to predecessor list!".format(connection_active["connectingRoad"]))
                                junction_roads_list.append(connection_active["connectingRoad"])
                        
                        #the predecessors are junction-roads which only have one lane. Easy to identify.
                        pre_roads_dict=dict()
                        if(len(junction_roads_list)==1):
                            pre_road_id=junction_roads_list[0]
                            print("Getting predecessor for merge, road id is {},  pre_road id is {}".format(road_id, pre_road_id))
                            pre_lane_id=road_list.roads[pre_road_id].getLaneByOpenDriveID(-1)
                            if(pre_lane_id is None):
                                pre_lane_id=road_list.roads[pre_road_id].getLaneByOpenDriveID(1)
                                if(pre_lane_id is None):
                                    raise Exception("ERROR: No lane id for road {} junction predecessor road {} found!".format(road_id, pre_road_id))
                            print("Found lane id {} ".format(pre_lane_id))
                            pre_lane=road_list.lanes[pre_lane_id]
                            pre_roads_dict[ManeuverState.MERGE]=pre_lane_id
                            lane_id=road_list.roads[road_id].getLaneByOpenDriveID(-1)
                            if(lane_id is None):
                                raise Exception("ERROR: road {} has junction {} as predecessor, but no right lane!".format(road_id, pre_id))
                            road_list.setLaneSuccessorDict(lane_id, pre_roads_dict)                                
                        elif(len(junction_roads_list)>1):
                            for pre_road_id in junction_roads_list:
                                #Junction-Roads do only have one lane, so we do not need to lookup them but just guess.
                                pre_lane_id=road_list.roads[pre_road_id].getLaneByOpenDriveID(-1)
                                if(pre_lane_id is None):
                                    pre_lane_id=road_list.roads[pre_road_id].getLaneByOpenDriveID(1)
                                    if(pre_lane_id is None):
                                        raise Exception("ERROR: No lane id for road {} junction predecessor road {} found!".format(road_id, pre_road_id))
                                pre_lane=road_list.lanes[pre_lane_id]
                                #print("Adding lane {} with angle change {}".format(pre_lane_id, pre_lane.getAngleChange()))
                                if(pre_lane.getAngleChange()>angle_left_threshold):
                                    #the angles are reverted here!
                                    if(ManeuverState.LEFT in pre_roads_dict):
                                        raise Exception("ERROR: road {} junction predecessor {} road has already a LEFT entry! {}".format(road_id, pre_id, pre_roads_dict))
                                    #print("Adding {} as left for {}!".format())
                                    pre_roads_dict[ManeuverState.LEFT]=pre_lane_id
                                elif(pre_lane.getAngleChange()<angle_right_threshold):
                                    if(ManeuverState.RIGHT in pre_roads_dict):
                                        raise Exception("ERROR: road {} junction predecessor {} road has already a RIGHT entry! {}".format(road_id, pre_id, pre_roads_dict))
                                    pre_roads_dict[ManeuverState.RIGHT]=pre_lane_id                                        
                                else:
                                    if(ManeuverState.STRAIGHT in pre_roads_dict):
                                        raise Exception("ERROR: road {} junction predecessor {} road has already a STRAIGHT entry! {}".format(road_id, pre_id, pre_roads_dict))
                                    pre_roads_dict[ManeuverState.STRAIGHT]=pre_lane_id
                        else:
                            raise Exception("ERROR Road {} junction {} predecessor has no valid roads!".format(road_id, pre_id))
                        #The Predecessor entry for a junction is always for the left lane from opendrive.
                        lane_id=road_list.roads[road_id].getLaneByOpenDriveID(1)
                        if(lane_id is None):
                            raise Exception("ERROR: road {} has junction {} as predecessor, but no right lane!".format(road_id, pre_id))
                        road_list.setLaneSuccessorDict(lane_id, pre_roads_dict)
                    else:
                        raise Exception("ERROR: Road {} prececessor elementType is invalid!".format(road_id))      

        for key,lane_suc in road_list.successors.items():
            if(lane_suc is None):
                raise Exception("Lane {} successors is none!".format(key))
            
        for key, lane_pre in road_list.predecessors.items():
            if(lane_pre is None):
                print("Lane {} predecessor list is None, generating...".format(key))
                pre_roads_dict=dict()
                pre_roads_list=list()
                for key2, lane_suc_dict in road_list.successors.items():
                    for key3, lane_suc in lane_suc_dict.items():
                        if(lane_suc == key):
                            pre_roads_list.append(key2)

                if(len(pre_roads_list)==1):
                    pre_roads_dict[ManeuverState.NEXT] = pre_roads_list[0]
                else:
                    for lane_suc in pre_roads_list:
                        pre_lane=road_list.lanes[lane_suc]
                        print("Adding lane {} with angle change {}".format(lane_suc, pre_lane.getAngleChange()))
                        if(pre_lane.getAngleChange()<-angle_left_threshold):
                            #the angles are reverted here!
                            if(ManeuverState.LEFT in pre_roads_dict and pre_roads_dict[ManeuverState.LEFT]!=lane_suc):
                                raise Exception("ERROR: lane {} predecessor has already a LEFT entry! {}".format(key,  pre_roads_dict))
                            #print("Adding {} as left for {}!".format())
                            pre_roads_dict[ManeuverState.LEFT]=lane_suc
                        elif(pre_lane.getAngleChange()>-angle_right_threshold):
                            if(ManeuverState.RIGHT in pre_roads_dict and pre_roads_dict[ManeuverState.RIGHT]!=lane_suc):
                                raise Exception("ERROR: lane {} predecessor has already a RIGHT entry! {}".format(key,  pre_roads_dict))
                            pre_roads_dict[ManeuverState.RIGHT]=lane_suc                                        
                        else:
                            if(ManeuverState.STRAIGHT in pre_roads_dict and pre_roads_dict[ManeuverState.STRAIGHT]!=lane_suc):
                                print("ERROR: lane {} predecessor lane {} has already a STRAIGHT entry (could be merge lane)! {}".format(key, lane_suc, pre_roads_dict))
                            else:
                                pre_roads_dict[ManeuverState.STRAIGHT]=lane_suc
                print("Generated: {}".format(pre_roads_dict))
                road_list.setLanePredecessorDict(key, pre_roads_dict)

        # print()
        # print("Successors:")
        # for suc in road_list.successors.items():
        #     print(suc)
        
        # print()
        # print("Predecessors:")
        # for pre in road_list.predecessors.items():
        #     print(pre)



        print("INFO: Overall road network length is {0}m.".format(overall_len))
        print("INFO: DONE!")
    
    return road_list