from LITD_RoadList import *

class RoadManager:
    def __init__(self):
        self.road_list=RoadList()
        return

    def addRoad(self, id, length, junction=None, name=None):
        return

    def ReadOpenDrive(self, xml_tree, scaling_factor=1, lane_offset=0.25):
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

        print(junction_dict)
        # roads is a list of "road" tags
        print("INFO: Loading roads...")
        roads = root_node.findall("road") 

        if(roads is None):
            raise Exception("ERROR: OpenDRIVE has no roads!")
        else:
            print("INFO: OpenDRIVE has {} roads!\nLoading...".format(len(roads)))
            overall_len=0.0
            overall_lanes=0
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
                        #We ignore the contactPoint, because our polynomials should always run in the right direction and we only have roads as types.
                        pre_type=road_link_pre.get("elementType")
                        pre_id=road_link_pre.get("elementId")
                        pre_contact_point=road_link_pre.get("contactPoint")
                        if(pre_type is None or pre_contact_point is None):
                            raise Exception("ERROR: Road {} predecessor has no elementType or contactPoint".format(road_id))
                        
                        if(pre_type.lower()=="road"):
                            #construct a default follower
                            predecessors_dict={RoadDecisions.NEXT:-100}
                        elif(pre_type.lower()=="junction"):
                            #construct a decision
                            predecessors_dict={RoadDecisions.LEFT: -100, RoadDecisions.STRAIGHT: -100, RoadDecisions.RIGHT: -100}
                        else:
                            raise Exception("ERROR: Road {} predecessor elementType is invalid!".format(road_id))
                        
                        
                        
                    
                    #process the sucessor tag
                    road_link_suc=road_link.find("successor")
                    if(road_link_suc is None):
                        print("WARNING: Road {} link has no successor-tag!".format(road_id))
                                
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
                    raise Exception("Road {} PlanView has no Geometry entry!".format(road_id))
                    
                for geo in geometries:

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
                        
                if(road_junction>=0):
                    lep3=LaneElementPoly3.fromOpenDrive(pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y)
                    self.road_list.addLaneElement(lep3, new_lane_id, None, None)
                    new_lane_id+=1
                else:
                    lanes = road.find("lanes")
                    #print("Road {} has {} lanes".format(road_id,len(lanes)))
                    for laneOffset in lanes.findall("laneOffset"):
                        sPos = float(laneOffset.get("s"))
                        a = float(laneOffset.get("a"))
                        b = float(laneOffset.get("b"))
                        c = float(laneOffset.get("c"))
                        d = float(laneOffset.get("d"))
                        if(abs(a)>0.001 or abs(b)>0.001 or abs(c)>0.001 or abs(d)>0.001):
                            print("WARNING: Road {} has an laneOffset that is important! Ignoring!".format(road_id))
                        
                    lane_sections=lanes.findall("laneSection")
                    if(lane_sections is None or len(lane_sections)>1):
                        print("WARNING: Road {} has multiple lane sections!".format(road_id))
                    
                    
                    #could be replaced by a for-each loop, if required...
                    lane_section=lane_sections[0]
                    
                    section_left=lane_section.find("left")
                    if(section_left is not None):
                        lanes_left=section_left.findall("lane")
                        if(lanes_left is None or len(lanes_left)!=1):
                            raise Exception("ERROR: in Road {}, Multiple left lane elements per lane are not supported!".format(road_id))
                        lane_left=lanes_left[0]
                        #lane_left_id=
                        lep3=LaneElementPoly3.fromOpenDrive(pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y)
                        lep3.addOffsetAndFit(lane_offset, True)
                        self.road_list.addLaneElement(lep3, new_lane_id, None, None)
                        new_lane_id+=1
                    else:
                        print("WARNING: Road {} has a laneSection without a left lane!".format(road_id))
                        
                    section_right=lane_section.find("right")
                    if(section_right is not None):
                        lanes_right=section_right.findall("lane")
                        if(lanes_right is None or len(lanes_right)!=1):
                            raise Exception("ERROR: in Road {}, Multiple right lane elements per lane are not supported!".format(road_id))
                        lane_right=lanes_right[0]
                        lep3=LaneElementPoly3.fromOpenDrive(pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y)
                        lep3.addOffsetAndFit(lane_offset, False)
                        self.road_list.addLaneElement(lep3, new_lane_id, None, None)
                        new_lane_id+=1
                    else:
                        print("WARNING: Road {} has a laneSection without a right lane!".format(road_id))
                        
            
                            
                    
                            
                    #lep3=LaneElementPoly3.fromOpenDrive(pp3_aU, pp3_bU, pp3_cU, pp3_dU, pp3_aV, pp3_bV, pp3_cV, pp3_dV, geo_hdg, geo_x, geo_y)
                    #self.road_list.addLaneElement(lep3, new_lane_id, None, None)
                    #new_lane_id+=1
                            
                    
                    


            print("Overall road network length is {0}m.".format(overall_len))
        