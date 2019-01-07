package org.usfirst.frc.team687.robot.constants;

import org.usfirst.frc.team687.robot.constants.DriveConstants;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.modifiers.TankModifier;
public class AutoConstants {

    // pathfinder constants
    public static final double dt = 0.02;
    public static final double kAcceleration = 8;
    public static final double kCruiseVelocity = 5;
    // Jerk is set to a high number since jerk barely matters, poofs don't jerk anymore
    public static final double kJerk = 100;

    private static Config test_config = new Config(Trajectory.FitMethod.HERMITE_CUBIC, Config.SAMPLES_HIGH, dt, kCruiseVelocity, kAcceleration, kJerk);
    private static Waypoint[] test_points = new Waypoint[] {
        new Waypoint(0, 0, 0),
        new Waypoint(5, -5, 0)  
    };
    public static Trajectory testTraj = Pathfinder.generate(test_points, test_config);
    
}