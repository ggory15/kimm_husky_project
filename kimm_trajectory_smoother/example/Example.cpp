#include <iostream>
#include <cstdio>
#include <Eigen/Core>
#include "Trajectory.h"
#include "Path.h"

using namespace std;
using namespace Eigen;

using namespace kimmtraj;

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_2d_planar )
{
	stdlist_Eigenvec waypoints;
	VectorXd waypoint(3);
	waypoint << 0.0, 0.0, 0.0;
	waypoints.extend(waypoint);
	waypoint << 0.0, 0.2, 1.0;
	waypoints.extend(waypoint);
	waypoint << 0.0, 3.0, 0.5;
	waypoints.extend(waypoint);
	waypoint << 1.1, 2.0, 0.0;
	waypoints.extend(waypoint);
	waypoint << 1.0, 0.0, 0.0;
	waypoints.extend(waypoint);
	waypoint << 0.0, 1.0, 0.0;
	waypoints.extend(waypoint);
	waypoint << 0.0, 0.0, 1.0;
	waypoints.extend(waypoint);

	VectorXd maxAcceleration(3);
	maxAcceleration << 1.0, 1.0, 1.0;
	VectorXd maxVelocity(3);
	maxVelocity << 1.0, 1.0, 1.0;

	Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
	trajectory.outputPhasePlaneTrajectory();
	if(trajectory.isValid()) {
		double duration = trajectory.getDuration();
		cout << "Trajectory duration: " << duration << " s" << endl << endl;
		cout << "Time      Position                  Velocity" << endl;
		for(double t = 0.0; t < duration; t += 0.1) {
			printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", t, trajectory.getPosition(t)[0], trajectory.getPosition(t)[1], trajectory.getPosition(t)[2],
				trajectory.getVelocity(t)[0], trajectory.getVelocity(t)[1], trajectory.getVelocity(t)[2]);
		}
		printf("%6.4f   %7.4f %7.4f %7.4f   %7.4f %7.4f %7.4f\n", duration, trajectory.getPosition(duration)[0], trajectory.getPosition(duration)[1], trajectory.getPosition(duration)[2],
			trajectory.getVelocity(duration)[0], trajectory.getVelocity(duration)[1], trajectory.getVelocity(duration)[2]);
	}
	else {
		cout << "Trajectory generation failed." << endl;
	}

	string s;
	cin >> s;

}
BOOST_AUTO_TEST_SUITE_END ()
