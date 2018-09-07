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

import java.lang.reflect.Array;
import java.util.Arrays;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.utilities.TrajectoryUtils;

public class DrivePurePursuit extends Command {

  private Trajectory m_trajectory;
  private double m_lookahead, m_x1, m_x2, m_y1, m_y2,
  m_a, m_b, m_c, m_goalX1, m_goalX2, m_goalY1, m_goalY2,
   m_startTime, m_time, m_robotX, m_robotY, m_slope, 
   m_yInt, m_error1, m_error2, m_angle, m_targetAngle1, m_targetAngle2;
  private Segment m_currentSegment, m_segment2;
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
    m_robotX = Robot.drive.getXpos();
    m_robotY = Robot.drive.getYpos();
    m_currentSegment = TrajectoryUtils.getClosestSegment(m_robotX, m_robotY, m_trajectory, m_index, 5);
    m_index = Arrays.asList(m_trajectory).indexOf(m_currentSegment);
    m_segment2 = TrajectoryUtils.getCloserSegment(m_robotX, m_robotY, m_currentSegment, m_segment2);
    
    m_x1 = m_currentSegment.x;
    m_y1 = m_currentSegment.y;
    m_x2 = m_segment2.x;
    m_y2 = m_segment2.y;
    m_slope = (m_y2 - m_y1) / (m_x2 - m_x1);
    m_yInt = m_y2 - m_slope * m_x2;
    
    // quadratic formula, solve for intercept of line from path and lookahead circle from robot
    m_a = (1 + m_slope * m_slope);
    m_b = (-2 * m_robotX) + (2 * m_slope * (m_yInt - m_robotY));
    m_c = (m_robotX * m_robotX) + Math.pow((m_yInt - m_robotY), 1) - m_lookahead * m_lookahead;

    m_goalX1 = (-m_b - Math.sqrt(m_b * m_b - 4 * m_a * m_c)) / (2 * m_a); 
    m_goalX1 = (-m_b + Math.sqrt(m_b * m_b - 4 * m_a * m_c)) / (2 * m_a); 
    m_goalY1 = m_slope * m_goalX1 + m_yInt;
    m_goalY2 = m_slope * m_goalX2 + m_yInt;

    m_angle = -(360 - Robot.drive.getRawYaw()) % 360;
    
    m_targetAngle1 = Math.toDegrees(Math.atan2(m_goalX1 - m_robotX, m_goalY1 - m_robotY));

    m_error1 = m_targetAngle1 - m_angle;
    if (m_error1 >= 180) {
      m_error1 -= 360;
    }
    if (m_error1 <= -180) {
      m_error1 += 360;
    }

    m_targetAngle1 = Math.toDegrees(Math.atan2(m_goalX1 - m_robotX, m_goalY1 - m_robotY));

    m_error2 = m_targetAngle2 - m_angle;
    if (m_error2 >= 180) {
      m_error2 -= 360;
    }
    if (m_error2 <= -180) {
      m_error2 += 360;
    }

    
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
