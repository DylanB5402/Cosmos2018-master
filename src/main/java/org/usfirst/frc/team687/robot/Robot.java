/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot;

import com.nerdherd.lib.misc.AutoChooser;
import com.nerdherd.lib.sensor.UltrasonicSensor;

import org.usfirst.frc.team687.robot.subsystems.Arm;
import org.usfirst.frc.team687.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */

public class Robot extends TimedRobot {
	public static PowerDistributionPanel pdp;
	public static Arm arm;
	public static OI oi;
	public static final String kDate = "2019_01_12_";
	public static AutoChooser chooser;
	public static Drive drive;
	public static UltrasonicSensor ultrasonic;



	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		pdp = new PowerDistributionPanel();
		arm = new Arm();
		chooser = new AutoChooser();
		drive = new Drive();
		ultrasonic = new UltrasonicSensor(0);
		oi = new OI();
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
		Robot.arm.reportState();
		SmartDashboard.putData(pdp);
		Robot.arm.stopLog();
		drive.stopLog();
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
		Robot.arm.reportState();
		SmartDashboard.putData(pdp);
	}

	
	@Override
	public void autonomousInit() {
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
		Robot.arm.reportState();
		SmartDashboard.putData(pdp);
		Robot.arm.startLog();
		drive.startLog();
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
		Robot.arm.reportState();
		SmartDashboard.putData(pdp);
		Robot.arm.logToCSV();
		drive.logToCSV();

	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
		Robot.arm.reportState();
		SmartDashboard.putData(pdp);

	}
}

