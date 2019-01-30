/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot.subsystems;

import com.nerdherd.lib.drivetrain.singlespeed.Drivetrain;
import com.nerdherd.lib.motor.NerdyTalon;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Drive extends Drivetrain {

    
    public Drive() {
        
        super(RobotMap.kLeftMasterTalonID, RobotMap.kRightMasterTalonID, 	    
        new NerdyTalon[]{
        new NerdyTalon(RobotMap.kLeftSlaveTalon1ID),
        new NerdyTalon(RobotMap.kLeftSlaveTalon2ID)}, 	  
        new NerdyTalon[]{new NerdyTalon(RobotMap.kRightSlaveTalon1ID),
        new NerdyTalon(RobotMap.kRightSlaveTalon2ID)}, 	
        true, false);

        super.configAutoChooser(Robot.chooser);
        super.configMaxVelocity(30000);
        super.configSensorPhase(false, false);
        
        super.configTicksPerFoot(17000, 17000);
        super.configDate("2019_1_26_");
        // floor
        super.configLeftPIDF(0.05, 0, 0, 0.028004625);
        super.configRightPIDF(0.05, 0, 0, 0.030084725);
        super.configStaticFeedforward(1.152, 1.228);

        // cart
        // drive.configLeftPIDF(0.05, 0, 0, 0.026995605);
        // drive.configRightPIDF(0.05, 0, 0, 0.026487175);
        // drive.configStaticFeedforward(0.760, 1.386);
    }

    @Override
    public void periodic() {
        super.reportToSmartDashboard();
        super.calcXY();
    }
}
