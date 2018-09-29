import sys
import argparse
from lxml import etree
from LITD_RoadManager import RoadManager

if __name__ == "__main__":
    print("starting lane_planner testing program...")
    parser = argparse.ArgumentParser(description="Runs the LITD Lane-Planner offline converter module. Reads the OpenDRIVE file, allows editing and converts it to the LITD Lane-Planner format.")
    parser.add_argument("output_prefix", help="The output file prefix. The output files (maybe multiple) will be called <arg>_<part>.csv)", nargs=1)
    parser.add_argument("xodr_file", nargs=1, help="The input OpenDRIVE File")
    parser.add_argument("scaling_numerator", nargs=1, help="The numerator of the scaling factor (num:denum)")
    parser.add_argument("scaling_denumerator", nargs=1, help="The denumerator of the scaling factor")
    parser.add_argument("lane_offset", nargs=1, help="The offset of the lane-center from the center-line.")

    args=parser.parse_args()

    num=1.0
    denum=1.0
    try:
        num=float(args.scaling_numerator[0])
        denum=float(args.scaling_denumerator[0])
        lane_offset=float(args.lane_offset[0])
    except ValueError:
        print("ERROR: Scaling numerator, denumerator and the lane offset must be floats! Given: {}:{}".format(args.scaling_numerator[0], args.scaling_denumerator[0]))
        exit(1)

    if(num==0  or denum==0):
        print("ERROR: Scaling numerator or denumerator is 0!")
        exit(1)
    if(lane_offset<=0):
        print("ERROR: lane offset must be greater then 0!")
        exit(1)
    
    scaling_factor=num/denum
    if(scaling_factor<0.05 or scaling_factor>50):
        print("WARNING: Scaling factor is {}. Is this wanted? y/[N]".format(scaling_factor))
        in_char=sys.stdin.read(1)
        if(in_char==89 or in_char==121):
            print("Continuing!")
        else:
            print("Exiting...")
            exit(1)
        



    #openDrive = None
    xml_tree=None
    with open(args.xodr_file[0],"r") as od_file:
        print("Reading from \"{0}\"".format(args.xodr_file[0]))
        xml_tree=etree.parse(od_file)
        #openDrive = parse_opendrive(xml_tree.getroot())

    if(xml_tree is None):
        print("OpenDRIVE not opened, exiting...")

    rm = RoadManager()

    rm.ReadOpenDrive(xml_tree, scaling_factor, 0.25)              
            
    exit(0)
else:
    print("This is not a library!")
    exit(1)