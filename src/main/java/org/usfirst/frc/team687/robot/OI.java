/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot;

import org.usfirst.frc.team687.robot.commands.arm.ArmSetAngle;
import org.usfirst.frc.team687.robot.commands.arm.ArmSetPosition;
import org.usfirst.frc.team687.robot.commands.arm.ResetArmEncoder;
import org.usfirst.frc.team687.robot.commands.arm.SetArmPower;
import org.usfirst.frc.team687.robot.commands.drive.ResetGyro;
import org.usfirst.frc.team687.robot.commands.drive.auto.DriveOpenLoop;
import org.usfirst.frc.team687.robot.commands.intake_wheel.SetBigIntakePower;
import org.usfirst.frc.team687.robot.commands.intake_wheel.SetLittleIntakePower;
import org.usfirst.frc.team687.robot.commands.intake_wheel.StopBothRollers;
import org.usfirst.frc.team687.robot.constants.ArmConstants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
	
	public Joystick leftStick;
	public Joystick rightStick;
	public Joystick armStick;
	private SendableChooser directionChooser;
	public JoystickButton intakeTop_1, intakeBottom_2, stopBothIntakes_3, outtakeTop_4, outtakeBottom_5;
	public JoystickButton downPos_8, upPos_7, horizontalPos_9 ;

	public OI() {
		leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		armStick = new Joystick(2);

		intakeTop_1 = new JoystickButton(armStick, 1);
		intakeBottom_2 = new JoystickButton(armStick, 2);
		stopBothIntakes_3 = new JoystickButton(armStick, 3);
		outtakeTop_4 = new JoystickButton(armStick, 4);
		outtakeBottom_5 = new JoystickButton(armStick, 5);
		upPos_7 = new JoystickButton(armStick, 7);
		downPos_8 = new JoystickButton(armStick, 8);
		horizontalPos_9 = new JoystickButton(armStick, 9);

		intakeTop_1.whenPressed(new SetBigIntakePower(-0.25));
		intakeBottom_2.whenPressed(new SetLittleIntakePower(-0.25));
		stopBothIntakes_3.whenPressed(new StopBothRollers());
		outtakeTop_4.whenPressed(new SetBigIntakePower(0.25));
		outtakeBottom_5.whenPressed(new SetLittleIntakePower(0.25));
		upPos_7.whenPressed(new ArmSetAngle(45));
		downPos_8.whenPressed(new ArmSetAngle(5));
		horizontalPos_9.whenPressed(new ArmSetAngle(0));

		SmartDashboard.putData("Reset Gyro", new ResetGyro());

		SmartDashboard.putData("Arm -12V" , new SetArmPower(-1));
		SmartDashboard.putData("Arm -9V" , new SetArmPower(-0.75));
		SmartDashboard.putData("Arm -6V" , new SetArmPower(-0.5));
		SmartDashboard.putData("Arm -3V" , new SetArmPower(-0.25));
		SmartDashboard.putData("Arm 0V" , new SetArmPower(0));
		SmartDashboard.putData("Arm 3V" , new SetArmPower(0.25));
		SmartDashboard.putData("Arm 6V" , new SetArmPower(0.5));
		SmartDashboard.putData("Arm 9V" , new SetArmPower(0.75));
		SmartDashboard.putData("Arm 1V" , new SetArmPower(1));

		SmartDashboard.putData("Arm 0 degrees", new ArmSetAngle(0));
		SmartDashboard.putData("Arm 10 degrees", new ArmSetAngle(10));
		SmartDashboard.putData("Arm 45 degrees", new ArmSetAngle(45));
		SmartDashboard.putData("Arm Up pos", new ArmSetPosition(ArmConstants.kArmUpPos));
		SmartDashboard.putData("Stop both rollers", new StopBothRollers());
		
		SmartDashboard.putData("Big Intake -12V" , new SetBigIntakePower(-1));
		SmartDashboard.putData("Big Intake -9V" , new SetBigIntakePower(-0.75));
		SmartDashboard.putData("Big Intake -6V" , new SetBigIntakePower(-0.5));
		SmartDashboard.putData("Big Intake -3V" , new SetBigIntakePower(-0.25));
		SmartDashboard.putData("Big Intake 12V" , new SetBigIntakePower(1));
		SmartDashboard.putData("Big Intake 9V" , new SetBigIntakePower(0.75));
		SmartDashboard.putData("Big Intake 6V" , new SetBigIntakePower(0.5));
		SmartDashboard.putData("Big Intake 3V" , new SetBigIntakePower(0.25));
		SmartDashboard.putData("Big Intake 0V" , new SetBigIntakePower(0));
		
		SmartDashboard.putData("Little Intake -12V" , new SetLittleIntakePower(-1));
		SmartDashboard.putData("Little Intake -9V" , new SetLittleIntakePower(-0.75));
		SmartDashboard.putData("Little Intake -6" , new SetLittleIntakePower(-0.5));
		SmartDashboard.putData("Little Intake -3V" , new SetLittleIntakePower(-0.25));
		SmartDashboard.putData("Little Intake 0V" , new SetLittleIntakePower(0));
		SmartDashboard.putData("Little Intake 3V" , new SetLittleIntakePower(0.25));
		SmartDashboard.putData("Little Intake 6V" , new SetLittleIntakePower(0.5));
		SmartDashboard.putData("Little Intake 9V" , new SetLittleIntakePower(0.75));
		SmartDashboard.putData("Little Intake 12V" , new SetLittleIntakePower(1));
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
		// SmartDashboard.putData("Pf Test", new DriveTrajectory("right_to_right_switch_back"));
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
