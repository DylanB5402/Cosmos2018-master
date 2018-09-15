package org.usfirst.frc.team687.robot.constants;

import org.usfirst.frc.team687.robot.constants.DriveConstants;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Config;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.modifiers.TankModifier;
public class AutoConstants {

    private static Config test_config = new Config(Trajectory.FitMethod.HERMITE_CUBIC, Config.SAMPLES_HIGH, 0.02, 13, 9, 100);
    private static Waypoint[] test_points = new Waypoint[] {
        new Waypoint(0, 0, Pathfinder.d2r(90)),
        new Waypoint(5, 7, Pathfinder.d2r(90))  
    };
    private static Trajectory test = Pathfinder.generate(test_points, test_config);
    

    
}