/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.PurePursuitController;
import org.usfirst.frc.team687.robot.Robot;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;

public class DrivePurePursuit extends Command {

  private PurePursuitController m_controller;
  private double m_leftVelocity, m_rightVelocity;

  public DrivePurePursuit(Trajectory traj, double lookahead, boolean goingForward) {
    m_controller = new PurePursuitController(traj, lookahead, goingForward, DriveConstants.kDrivetrainWidth);
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_controller.calculate(Robot.drive.getXpos(), Robot.drive.getYpos(), Robot.drive.getRawYaw());
    m_leftVelocity = m_controller.getLeftVelocity();
    m_rightVelocity = m_controller.getRightVelocity();
    Robot.drive.setVelocityFPS(m_leftVelocity, m_rightVelocity);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_controller.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
