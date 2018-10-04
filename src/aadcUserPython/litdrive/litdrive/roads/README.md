# OpenDRIVE to LITD Lane Format converter

   * LITD_RoadManager.py has helper functions and a OpenDRIVE to RoadList converter function
   * LITD_RoadList.py has all data-structures for lane generation.

To use and convert an OpenDRIVE, use the ipython/jupyter notebook or ask Georg Zachl (aadc@zachlge.org).

WARNING: The orignal OpenDRIVE has been modified, because the merge-lane has some other offset handling then normal roads. I fixed this by moving the road.
ALSO: The creation of the merge-lane successors and predecessors does not work correctly, as it is a real corner case. Implementing it would be way to much work and does not pay off.
SO: this has to be fixed by hand. See the ipython/jupyter notebook for that.