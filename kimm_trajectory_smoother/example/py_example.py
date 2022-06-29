import numpy as np
import kimm_trajectory as traj
import eigenpy

waypoints = traj.stdlist()
waypoint = np.array([0, 0, 0])
waypoints.extend(waypoint)
waypoint = np.array([0, 0.2, 0])
waypoints.extend(waypoint)
waypoint = np.array([0, 0.2, 0.5])
waypoints.extend(waypoint)

path = traj.Path(waypoints, 0.1)
maxAcceleration = np.array([100, 100, 100])
maxVelocity = np.array([100, 100, 100])

trajectory = traj.Trajectory(path, maxVelocity, maxAcceleration, 0.1)
trajectory.outputPhasePlaneTrajectory()

if (trajectory.isValid()):
    duration = trajectory.getDuration()
    print ("Trajectory duration: ", duration)

    for t in np.arange(0, duration, 0.1):
        print (trajectory.getPosition(t).transpose())
    print (trajectory.getPosition(duration).transpose())

