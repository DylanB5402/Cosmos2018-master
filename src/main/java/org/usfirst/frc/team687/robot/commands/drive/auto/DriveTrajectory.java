/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot.commands.drive.auto;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.constants.AutoConstants;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.followers.DistanceFollower;

public class DriveTrajectory extends Command {
  private Trajectory m_leftTrajectory, m_rightTrajectory, m_sourceTrajectory;
  private DistanceFollower m_leftFollower, m_rightFollower;
  private double m_leftOutput, m_rightOutput, m_turn, m_angularError, m_angle,
  m_leftPosition, m_rightPosition, m_sign;
  private TankModifier m_modifier;
  private boolean m_forwards;

  public DriveTrajectory(Trajectory traj, boolean forwards) {
    m_sourceTrajectory = traj;
    m_forwards = forwards;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_modifier = new TankModifier(m_sourceTrajectory);
    m_modifier.modify(DriveConstants.kDrivetrainWidth);  

    if (m_forwards) {
      m_leftTrajectory = m_modifier.getLeftTrajectory();
      m_rightTrajectory = m_modifier.getRightTrajectory();
      m_sign = 1;
    }
    else {
      m_leftTrajectory = m_modifier.getRightTrajectory();
      m_rightTrajectory = m_modifier.getLeftTrajectory();
      m_sign = -1;
    }

    m_leftFollower = new DistanceFollower(m_leftTrajectory);
    m_rightFollower = new DistanceFollower(m_rightTrajectory);
    m_leftFollower.configurePIDVA(DriveConstants.kLeftVelocityP, 0, DriveConstants.kLeftVelocityD, DriveConstants.kLeftV, 0.003);
    m_rightFollower.configurePIDVA(DriveConstants.kRightVelocityP, 0, DriveConstants.kRightVelocityD, DriveConstants.kRightV, 0.003);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_angle = Robot.drive.getRawYaw();    
    m_leftPosition = Robot.drive.getLeftPositionFeet();
    m_rightPosition = Robot.drive.getLeftPositionFeet();
    m_angle = Robot.drive.getRawYaw();
    if (!m_forwards) {
      m_leftPosition = -m_leftPosition;
      m_rightPosition = -m_rightPosition;
    }
    m_leftOutput = m_sign * m_leftFollower.calculate(m_leftPosition) + DriveConstants.kLeftStatic;
    m_rightOutput = m_sign * m_rightFollower.calculate(m_rightPosition) + DriveConstants.kRightStatic;

    m_angularError = Pathfinder.boundHalfDegrees(-Pathfinder.r2d(m_leftFollower.getHeading()) - m_angle);
    m_turn = DriveConstants.kRotP * m_angularError;
    Robot.drive.addDesiredVelocities(m_leftFollower.getSegment().velocity, m_rightFollower.getSegment().velocity);
    Robot.drive.setPower(m_leftOutput + m_turn, m_rightOutput - m_turn);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return m_rightFollower.isFinished() && m_leftFollower.isFinished();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.setPowerZero();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
