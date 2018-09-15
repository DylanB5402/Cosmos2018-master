/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot.commands.drive.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

import java.lang.reflect.Array;
import java.util.Arrays;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.utilities.NerdyMath;
import org.usfirst.frc.team687.robot.utilities.TrajectoryUtils;
import org.usfirst.frc.team687.robot.constants.DriveConstants;

public class DrivePurePursuit extends Command {

  private Trajectory m_trajectory;
  private double m_lookahead, m_x1, m_x2, m_y1, m_y2,
  m_a, m_b, m_c, m_goalX1, m_goalX2, m_goalY1, m_goalY2,
   m_startTime, m_time, m_robotX, m_robotY, m_slope, 
   m_yInt, m_error1, m_error2, m_angle, m_targetAngle1,
    m_targetAngle2, m_goalX, m_goalY, m_xOffset, m_velocity, 
    m_innerVelocity, m_error, m_driveRadius;
  private double m_prevTime, m_leftDesiredVel, m_rightDesiredVel;
  private double m_leftError, m_rightError, m_leftPrevError, m_rightPrevError;
  private double m_leftVoltage, m_rightVoltage;
  private Segment m_currentSegment, m_segment2;
  private int m_index;
  private Boolean m_goingForwards;
  
  public DrivePurePursuit(Trajectory trajectory, double lookahead, Boolean goingForwards) {
    m_lookahead = lookahead;
    m_trajectory = trajectory;
    m_goingForwards = goingForwards;
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
    m_segment2 = TrajectoryUtils.getCloserSegment(m_robotX, m_robotY, m_trajectory.get(m_index-1),  m_trajectory.get(m_index + 1));
    
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

    m_targetAngle2 = Math.toDegrees(Math.atan2(m_goalX2 - m_robotX, m_goalY2 - m_robotY));

    m_error2 = m_targetAngle2 - m_angle;
    if (m_error2 >= 180) {
      m_error2 -= 360;
    }
    if (m_error2 <= -180) {
      m_error2 += 360;
    }

    if (Math.abs(m_error1) <= Math.abs(m_error2)) {
      m_error = m_error1;
      m_goalX = m_goalX1;
      m_goalY = m_goalY1;
    }
    else {
      m_error = m_error2;
      m_goalX = m_goalX2;
      m_goalY = m_goalY2;
    }

    m_xOffset = NerdyMath.distanceFormula(m_robotX, m_robotY, m_goalX, m_goalY) 
    * Math.cos(Math.toRadians(m_error));
    
    m_velocity = m_currentSegment.velocity;
    
    if (!m_goingForwards) {
      m_velocity *= -1;
    }
    
    if (m_xOffset > 0) {
      m_driveRadius = (m_lookahead * m_lookahead) / (2 * m_xOffset);
      m_innerVelocity = m_velocity * (m_driveRadius - (DriveConstants.kDrivetrainWidth/2)) / (m_driveRadius + (DriveConstants.kDrivetrainWidth/2));
      if (m_goingForwards) {
        if (Math.signum(m_error) == -1) {
          m_leftDesiredVel = m_innerVelocity;
          m_rightDesiredVel = m_velocity;
        }
        else {
          m_leftDesiredVel = m_velocity;
          m_rightDesiredVel = m_innerVelocity;
        }
      }
      else {
        if (Math.signum(m_error) == 1) {
          m_leftDesiredVel = m_innerVelocity;
          m_rightDesiredVel = m_velocity;
        }
        else {
          m_leftDesiredVel = m_velocity;
          m_rightDesiredVel = m_innerVelocity;
        }
      }
    }
    else {
      m_leftDesiredVel = m_velocity;
      m_rightDesiredVel = m_velocity;
    }

    // Replace with Talon Velocity Control down the line
    m_leftError = m_leftDesiredVel - Robot.drive.getLeftMasterSpeed();
    m_rightError = m_rightDesiredVel - Robot.drive.getRightMasterSpeed();

    m_leftVoltage = DriveConstants.kLeftStatic * Math.signum(m_leftDesiredVel) + DriveConstants.kLeftV * m_leftDesiredVel 
      + m_leftError * DriveConstants.kLeftVelocityP + DriveConstants.kLeftVelocityD
      * (m_leftError - m_leftPrevError)/(m_time - m_prevTime);
    m_rightVoltage = DriveConstants.kRightStatic * Math.signum(m_rightDesiredVel) + DriveConstants.kRightV * m_rightDesiredVel 
    + m_rightError * DriveConstants.kRightVelocityP + DriveConstants.kRightVelocityD
    * (m_rightError - m_rightPrevError)/(m_time - m_prevTime);
    Robot.drive.setVoltage(m_leftVoltage, m_rightVoltage);
    Robot.drive.addDesiredVelocities(m_leftDesiredVel, m_rightDesiredVel);
    m_prevTime = m_time;
    m_leftPrevError = m_leftError;
    m_rightPrevError = m_rightError;
    SmartDashboard.putNumber("left desired vel", m_leftDesiredVel);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // subtract 2, since arrays start at 0 and last segment has a velocity of 0
    return m_index == (m_trajectory.length() - 2);
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
