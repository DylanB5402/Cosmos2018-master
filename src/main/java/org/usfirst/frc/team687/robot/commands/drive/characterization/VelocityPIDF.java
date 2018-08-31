package org.usfirst.frc.team687.robot.commands.drive.characterization;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.constants.DriveConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VelocityPIDF implements Runnable {

    private double m_desiredVel, m_time, m_prevTime, m_startTime;
    private double m_leftError, m_rightError, m_leftPrevError, m_rightPrevError;
    private double m_leftVoltage, m_rightVoltage, m_desiredTime;
    private boolean m_started = false;

    public void setVelocity(double velocity) {
        m_desiredVel = velocity;
    }

    @Override
    public void run() {
        if (!m_started) {
            m_startTime = Timer.getFPGATimestamp();
            m_prevTime = Timer.getFPGATimestamp();
            m_leftPrevError = m_desiredVel - Robot.drive.getLeftMasterSpeed();
            m_rightPrevError = m_desiredVel - Robot.drive.getRightMasterSpeed();
            m_started = true;
        }

        m_time = Timer.getFPGATimestamp() - m_startTime;
        m_leftError = m_desiredVel - Robot.drive.getLeftMasterSpeed();
        m_rightError = m_desiredVel - Robot.drive.getRightMasterSpeed();

        m_leftVoltage = DriveConstants.kLeftStatic * Math.signum(m_desiredVel) + DriveConstants.kLeftV * m_desiredVel 
        + m_leftError * DriveConstants.kLeftVelocityP + DriveConstants.kLeftVelocityD
        * (m_leftError - m_leftPrevError)/(m_time - m_prevTime);
        m_rightVoltage = DriveConstants.kRightStatic * Math.signum(m_desiredVel) + DriveConstants.kRightV * m_desiredVel 
        + m_rightError * DriveConstants.kRightVelocityP + DriveConstants.kRightVelocityD
        * (m_rightError - m_rightPrevError)/(m_time - m_prevTime);
        // Robot.drive.setVoltage(m_leftVoltage, m_rightVoltage);
        // Robot.drive.addDesiredVelocities(m_desiredVel, m_desiredVel);
        m_prevTime = m_time;
        m_leftPrevError = m_leftError;
        m_rightPrevError = m_rightError;
        SmartDashboard.putString("VelocityPID", "Runnable is working");
    }

    public void stop() {
        m_desiredVel = 0;
    }

}