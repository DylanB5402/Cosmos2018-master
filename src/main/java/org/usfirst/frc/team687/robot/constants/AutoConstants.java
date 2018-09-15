package org.usfirst.frc.team687.robot.constants;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Pathfinder;

public class AutoConstants {

    public static Config test_config = new Config(Trajectory.FitMethod.HERMITE_CUBIC, Config.SAMPLES_HIGH, 0.02, 13, 9, 100);
    public static Waypoint[] test_points = new Waypoint[] {
        new Waypoint(0, 0, Pathfinder.d2r(90)),
        new Waypoint(5, 7, Pathfinder.d2r(90))
        
    };
    public static Trajectory test = Pathfinder.generate(test_points, test_config);

}