/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot;

import org.usfirst.frc.team687.robot.commands.drive.ResetDriveEncoders;
import org.usfirst.frc.team687.robot.commands.drive.ResetGyro;
import org.usfirst.frc.team687.robot.commands.drive.auto.DriveOpenLoop;
import org.usfirst.frc.team687.robot.commands.drive.auto.DriveTrajectory;
// import org.usfirst.frc.team687.robot.commands.drive.auto.DrivePurePursuit;
import org.usfirst.frc.team687.robot.commands.drive.characterization.DriveCharacterizationTest;
import org.usfirst.frc.team687.robot.commands.drive.characterization.VelocityTest;
import org.usfirst.frc.team687.robot.constants.AutoConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick leftStick;
	public Joystick rightStick;
	private SendableChooser directionChooser;

	public OI() {
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		SmartDashboard.putData("Reset Gyro", new ResetGyro());
		SmartDashboard.putData("Reset Drive Encoders", new ResetDriveEncoders());
		SmartDashboard.putData("Drive Characterization Test", new DriveCharacterizationTest(0.5));
		SmartDashboard.putData("Velocity Test", new VelocityTest(7000, 6));

		directionChooser = new SendableChooser<>();
		// directionChooser.addObject("Forward", "Forward");
		// directionChooser.addObject("Backwards", "Backwards");
		SmartDashboard.putData("Direction Chooser", directionChooser);
		SmartDashboard.putData("Open loop", new DriveOpenLoop());
		// SmartDashboard.putData("Pure pursuit Test", new DrivePurePursuit(AutoConstants.test, 1.5, true));
		// SmartDashboard.putString("Direction", getStartingDirection());
		SmartDashboard.putData("Pf Test", new DriveTrajectory("right_to_right_switch_back"));
	}
	
	public String getStartingDirection() {
		return (String) directionChooser.getSelected();
	}

	public double getLeftY() {
		return -leftStick.getY();
	}
	
	public double getLeftX() {
		return Math.pow(leftStick.getX(), 2) * Math.signum(leftStick.getX());
	}
	
	public double getRightY() {
		return -Math.pow(rightStick.getY(), 2) * Math.signum(rightStick.getY());	
	}
	
	public double getRightX() {
		return rightStick.getX();
	}
	
	public boolean isLeftTriggerPulled() {
		return leftStick.getRawButton(1);
	}
	
	public void reportToSmartDashboard() {
//		SmartDashboard.putNumber("Left Stick Y", getLeftY());
//		SmartDashboard.putNumber("Right Stick Y", getRightY());
	}
}
