/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot.commands.drive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

import org.usfirst.frc.team687.robot.Robot;

public class DrivePurePursuit extends Command {

  private Trajectory m_trajectory;
  private double m_lookahead, m_x1, m_x2, m_y1, m_y2, slope,
  m_a, m_b, m_c, m_goalX1, m_goalX2, m_goalY1, m_goalY2, m_startTime, m_time;
  private Segment m_currentSegment, m_segment_2;
  private int m_index;
  
  public DrivePurePursuit(Trajectory trajectory, double lookahead) {
    m_lookahead = lookahead;
    m_trajectory = trajectory;
    requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    m_startTime = Timer.getFPGATimestamp();
    m_index = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    m_time = Timer.getFPGATimestamp() - m_startTime;
    Segment seg = m_trajectory.get(1);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
