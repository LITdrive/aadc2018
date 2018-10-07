#include <cstdint>
#include <map>
#include <tuple>
#include <vector>


#include "stdafx.h"
#include <aadc_structs.h>

#include "LITD_VirtualPoint.h"



typedef enum {TLES_UNDRIVEN, TLES_CURRENT, TLES_PASSED} LITD_TrajectoryListEntryState;

class LITD_TrajectoryList {
    public:
        // Constructors
        LITD_TrajectoryList();
        
        //User functions
        bool addTrajectoryArray(tTrajectoryArray* trj);
        bool removeTrajectory(uint32_t id);

        uint32_t countTrajectories();

        //Returns the id of the dropped polynomial. if there should be multiple dropped, this one returns the newest dropped.
        uint32_t updateList();

        //Returns the nearest point to the given point on the known polynomials in target_pnt.
        //It updates the list, but does not drop the old items. 
        //Returns the ID of the last Tile, if we changed to a new one.
        uint32_t getDistanceToNearestPoint(LITD_VirtualPoint& car_pnt, LITD_VirtualPoint& target_pnt);

        //friend std::ostream& operator<<(std::ostream&, const LITD_TrajectoryList&);

        //std::tuple<uint32_t, double> getNextChangePoint();

        static void getPolyPoint(tTrajectory& trj, double p, LITD_VirtualPoint& pnt);


        static const double p_resolution;

    protected:
        std::map<uint32_t, std::tuple<tTrajectory, LITD_TrajectoryListEntryState>> t_map;
        static const uint32_t array_max_len=sizeof(((tTrajectoryArray *)0)->trajectories)/sizeof(tTrajectory);

        
};
