#include "LITD_TrajectoryList.h"
#include <list>
#include <cmath>
#include <cfloat>
#include "math_utilities.h"


const double LITD_TrajectoryList::p_resolution = 0.01;

LITD_TrajectoryList::LITD_TrajectoryList(): t_map() {
    
}

bool LITD_TrajectoryList::addTrajectoryArray(tTrajectoryArray* trj_array) {
    if(trj_array==NULL || trj_array->size>array_max_len) { return false; }
    for(uint32_t i=0; i<trj_array->size; i++) {
        tTrajectory* trj = &trj_array->trajectories[i];
        //if the id of an entry inside valid size is 0, we clear the trajectory list.
        if(trj->id==0) {
            t_map.clear();
        } else {
            t_map[trj->id]=std::tuple<tTrajectory, LITD_TrajectoryListEntryState>(*trj, TLES_UNDRIVEN);
        }
    }
    return true;
}

uint32_t LITD_TrajectoryList::countTrajectories() {
    return t_map.size();
}

bool LITD_TrajectoryList::removeTrajectory(uint32_t id) {
    if(t_map.erase(id)>0) {
        return true;
    }
    return false;
}

std::tuple<uint32_t, uint32_t, double> LITD_TrajectoryList::getDistanceToNearestPoint(LITD_VirtualPoint& car_pnt, LITD_VirtualPoint& target_pnt) {
    double smallest_dist=DBL_MAX;
    uint32_t smallest_id=0;
    uint32_t newest_passed_id=0;
    double p_at_smallest=-1.0;
    //Iterate over all map entries in a sorted manner (std::map is sorted!)
    for(auto it = t_map.begin(); it!= t_map.end(); it++) {

        //Extract tuple data. (Iterator has ->first is the key, ->second is the content)
        std::tuple<tTrajectory, LITD_TrajectoryListEntryState> tpl =  it->second;
        tTrajectory trj = std::get<0>(tpl);
        LITD_VirtualPoint poly_pnt;

        for(double j = trj.start; j<=trj.end; j += p_resolution) {
            getPolyPoint(trj, j, poly_pnt);
            double x_vec = poly_pnt.x - car_pnt.x;
            double y_vec = poly_pnt.y - car_pnt.y;
            //wrapTo2Pi(diff_heading_abs - wrapTo2Pi(vehicleTargetFrontAxlePosition.h))
            double angle_diff = wrapTo2Pi(wrapTo2Pi<double>(atan2(y_vec, x_vec)) - wrapTo2Pi<double>(car_pnt.h));

            if(angle_diff <= M_PI/2.0 || angle_diff >= 3.0/2.0 * M_PI) {
                double dist = sqrt(pow(x_vec, 2) + pow(y_vec, 2));
                if(dist<smallest_dist) {
                    smallest_id=trj.id;
                    smallest_dist=dist;

                    p_at_smallest=j;
                    target_pnt.x=poly_pnt.x;
                    target_pnt.y=poly_pnt.y;
                    target_pnt.h=poly_pnt.h;
                }
            }
        }
    }

    //We have passed all polys.
    if(smallest_id==0) {
        if(!t_map.empty()) {
            newest_passed_id=(t_map.end()--)->first;
            t_map.clear();
        } else {
            newest_passed_id=0;
        }
    } else {

        std::vector<uint32_t> to_delete;
        for(auto it = t_map.begin(); it!= t_map.end(); it++) {
            uint32_t index=it->first;
            std::tuple<tTrajectory, LITD_TrajectoryListEntryState> tpl =  it->second;
            LITD_TrajectoryListEntryState state = std::get<1>(tpl);
            if(index<smallest_id) {
                if(state!=TLES_PASSED) {
                    it->second = std::tuple<tTrajectory, LITD_TrajectoryListEntryState>(std::get<0>(tpl), TLES_PASSED);
                    newest_passed_id=index;
                }
                to_delete.push_back(index);
            } else if(index==smallest_id) {
                it->second = std::tuple<tTrajectory, LITD_TrajectoryListEntryState>(std::get<0>(tpl), TLES_CURRENT);
            } else {
                break;
            }
        }
        if(to_delete.size()>1) {
            to_delete.pop_back();
            for (auto it = to_delete.cbegin(); it != to_delete.cend(); it++) {
                t_map.erase(*it);
            }
        }
    }
    return std::tuple<uint32_t, uint32_t, double> (newest_passed_id, smallest_id, p_at_smallest);
}


void LITD_TrajectoryList::getPolyPoint(tTrajectory& trj, double p, LITD_VirtualPoint& pnt) {
    pnt.x = trj.dx*pow(p, 3) + trj.cx*pow(p, 2) + trj.bx*p + trj.ax;
    pnt.y = trj.dy*pow(p, 3) + trj.cy*pow(p, 2) + trj.by*p + trj.ay;

	double x_der = 3 * trj.dx*pow(p, 2) + 2 * trj.cx*p + trj.bx;
	double y_der = 3 * trj.dy*pow(p, 2) + 2 * trj.cy*p + trj.by;


    pnt.h = atan2(y_der, x_der);

    if(trj.backwards) {
        pnt.h += M_PI;
    }

    pnt.h = wrapTo2Pi(pnt.h);
}
