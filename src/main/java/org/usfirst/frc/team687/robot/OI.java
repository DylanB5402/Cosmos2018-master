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
import org.usfirst.frc.team687.robot.commands.drive.auto.DrivePurePursuit;
import org.usfirst.frc.team687.robot.commands.drive.characterization.DriveCharacterizationTest;
import org.usfirst.frc.team687.robot.commands.drive.characterization.VelocityTest;
import org.usfirst.frc.team687.robot.commands.intake_wheel.SetIntakeVoltage;
import org.usfirst.frc.team687.robot.commands.arm.ArmSetPosition;
import org.usfirst.frc.team687.robot.commands.arm.OpenLoopArm;
import org.usfirst.frc.team687.robot.constants.ArmConstants;
import org.usfirst.frc.team687.robot.constants.AutoConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team687.robot.commands.arm.ResetArmEncoder;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick leftStick;
	public Joystick rightStick;
	public Joystick armStick;
	private SendableChooser directionChooser;

	public OI() {
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		armStick = new Joystick(2);
		SmartDashboard.putData("Reset Gyro", new ResetGyro());


		SmartDashboard.putData("Big  Intake -12V" , new SetIntakeVoltage(-12));
		SmartDashboard.putData("Big Intake -10V" , new SetIntakeVoltage(-10));
		SmartDashboard.putData("Big Intake -8V" , new SetIntakeVoltage(-8));
		SmartDashboard.putData("Big Intake -6V" , new SetIntakeVoltage(-6));
		SmartDashboard.putData("Big Intake -4V" , new SetIntakeVoltage(-4));
		SmartDashboard.putData("Big Intake -2V" , new SetIntakeVoltage(-2));
		SmartDashboard.putData("Big Intake 0V" , new SetIntakeVoltage(0));
		SmartDashboard.putData("Big Intake 2V" , new SetIntakeVoltage(2));
		SmartDashboard.putData("Big Intake 4V" , new SetIntakeVoltage(4));
		SmartDashboard.putData("Big Intake 6V" , new SetIntakeVoltage(6));
		SmartDashboard.putData("Big Intake 8V" , new SetIntakeVoltage(8));
		SmartDashboard.putData("Big Intake 10V" , new SetIntakeVoltage(10));
		SmartDashboard.putData("Big Intake 12V" , new SetIntakeVoltage(12));
		// SmartDashboard.putData("Drive Characterization Test", new DriveCharacterizationTest(0.5));
		// SmartDashboard.putData("Velocity Test", new VelocityTest(7000, 6));
	
		SmartDashboard.putData("ResetArmEncoder", new ResetArmEncoder());
		

		directionChooser = new SendableChooser<>();
		// directionChooser.addObject("Forward", "Forward");
		// directionChooser.addObject("Backwards", "Backwards");
		SmartDashboard.putData("Direction Chooser", directionChooser);
		SmartDashboard.putData("Open loop", new DriveOpenLoop());
		// SmartDashboard.putData("Pure pursit Test", new DrivePurePursuit(AutoConstants.test, 1.5, true));
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
	
	public double getArmY() {
		return armStick.getY();
	}
	public boolean isLeftTriggerPulled() {
		return leftStick.getRawButton(1);
	}
	
	public void reportToSmartDashboard() {
//		SmartDashboard.putNumber("Left Stick Y", getLeftY());
//		SmartDashboard.putNumber("Right Stick Y", getRightY());
	}
}
