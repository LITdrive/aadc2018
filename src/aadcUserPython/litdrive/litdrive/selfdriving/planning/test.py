from litdrive.selfdriving.planning.planner import *

pl = Planner("../../../../../../configuration_files/maps/qualifying_2018_litd.pickle")

ids=[134, 189, 177, 183]
starts=[0.0, 0.0, 0.0, 0.0]
ends=[1.0, 1.0, 1.0, 1.0]
invert=[False, False, False, False]
backwards=[False, False, False, False]


print("Init!")
pl.initPlannerSettings(57)

print(pl)

man=[ManeuverState.LEFT, ManeuverState.LEFT, ManeuverState.STRAIGHT, ManeuverState.LEFT, ManeuverState.RIGHT]
for m in man:
    pl.addManeuver(m)



done_ids=[0,0,1,2,3,0,4,0,5,6,7,8,16]
current_ids=[0,1,2,3,4,4,5,4,6,7,8,9, 17]
current_ps=[0.0, 0.5, 0.5, 0.5, 0.5, 0.6, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]


for i in range(len(done_ids)):
    print("Loop {}:".format(i))
    buffer=pl.update(done_ids[i], current_ids[i], current_ps[i])
    print(pl, end="")
    print("Buffer:", end="")
    print(buffer, end="\n\n")


print("DONE!!!!!!!!")