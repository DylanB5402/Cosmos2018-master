package org.usfirst.frc.team687.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.RobotMap;
import org.usfirst.frc.team687.robot.commands.drive.teleop.ArcadeDrive;
import org.usfirst.frc.team687.robot.commands.drive.teleop.TankDrive;
import org.usfirst.frc.team687.robot.commands.drive.teleop.ClosedLoopTankDrive;
import org.usfirst.frc.team687.robot.constants.DriveConstants;
import org.usfirst.frc.team687.robot.utilities.NerdyTalon;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import org.usfirst.frc.team687.robot.commands.drive.characterization.VelocityPIDF;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;

/**
 *
 */
public class Drive extends Subsystem {

	private final NerdyTalon m_leftMaster, m_leftSlave1, m_leftSlave2;
	private final NerdyTalon m_rightMaster, m_rightSlave1, m_rightSlave2;
	private final AHRS m_nav;
	
	private double m_previousDistance, m_currentX, m_currentY, m_angleOffset, m_xOffset, m_yOffset;
    
    private String m_filePath1 = "/media/sda1/logs/";
	private String m_filePath2 = "/home/lvuser/logs/";
	private String m_fileName = "2018_09_15_pathfinder_test";
    private File m_file;
    public FileWriter m_writer;
    private boolean writeException = false;
	private double m_logStartTime = 0;
	private double m_leftDesiredVel, m_rightDesiredVel;
	private VelocityPIDF m_velocityPIDF;
	private Notifier m_velocityNotifier;
    
	public Drive() {
		
		m_nav = new AHRS(SPI.Port.kMXP);
		
		m_leftMaster = new NerdyTalon(RobotMap.kLeftMasterTalonSRXID);
		m_leftSlave1 = new NerdyTalon(RobotMap.kLeftSlaveTalonSRX1ID);
		m_leftSlave2 = new NerdyTalon(RobotMap.kLeftSlaveTalonSRX2ID);
		
		m_rightMaster = new NerdyTalon(RobotMap.kRightMasterTalonSRXID);
		m_rightSlave1 = new NerdyTalon(RobotMap.kRightSlaveTalonSRX1ID);
		m_rightSlave2 = new NerdyTalon(RobotMap.kRightSlaveTalonSRX2ID);
		
		
		m_leftMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		m_rightMaster.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
		
		m_rightSlave1.follow(m_rightMaster);
		m_rightSlave2.follow(m_rightMaster);
		
		m_leftSlave1.follow(m_leftMaster);
		m_leftSlave2.follow(m_leftMaster);
				
		m_leftMaster.setInverted(true);
		m_leftSlave1.setInverted(true);
		m_leftSlave2.setInverted(true);
		
		m_rightMaster.setInverted(false);
		m_rightSlave1.setInverted(false);
		m_rightSlave2.setInverted(false);
		m_leftMaster.setSensorPhase(false);
		m_rightMaster.setSensorPhase(false);
		
		// m_rightMaster.configPIDF(DriveConstants.kRightP, DriveConstants.kRightI, DriveConstants.kRightD, DriveConstants.kRightF, 0);
		// m_leftMaster.configPIDF(DriveConstants.kLeftP, DriveConstants.kLeftI, DriveConstants.kLeftD, DriveConstants.kLeftF, 0);

		// m_leftMaster.configMotionMagic(DriveConstants.kLeftAcceleration, DriveConstants.kLeftCruiseVelocity);
		// m_rightMaster.configMotionMagic(DriveConstants.kRightAcceleration, DriveConstants.kRightCruiseVelocity);

		m_leftMaster.setNeutralMode(NeutralMode.Brake);
		m_leftSlave1.setNeutralMode(NeutralMode.Brake);
		m_leftSlave2.setNeutralMode(NeutralMode.Brake);
		
		m_rightMaster.setNeutralMode(NeutralMode.Brake);
		m_rightSlave1.setNeutralMode(NeutralMode.Brake);
		m_rightSlave2.setNeutralMode(NeutralMode.Brake);
		
		m_rightMaster.configDefaultSettings();
		m_rightSlave1.configDefaultSettings();
		m_rightSlave2.configDefaultSettings();

		m_leftMaster.configDefaultSettings();
		m_leftSlave1.configDefaultSettings();
		m_leftSlave2.configDefaultSettings();

		m_leftMaster.configLinearUnits(DriveConstants.kTicksPerFoot);
		m_rightMaster.configLinearUnits(DriveConstants.kTicksPerFoot);
		
		m_velocityPIDF = new VelocityPIDF();
		m_velocityNotifier = new Notifier(m_velocityPIDF);
	}
	
	public void setPower(double leftPower, double rightPower) {

		m_leftMaster.set(ControlMode.PercentOutput, leftPower);
		m_rightMaster.set(ControlMode.PercentOutput, rightPower);
    }
	
	public void setVoltage(double leftVoltage, double rightVoltage) {
		m_leftMaster.set(ControlMode.PercentOutput, leftVoltage/12);
		m_rightMaster.set(ControlMode.PercentOutput, rightVoltage/12);
	}

	public void setPowerZero() {
		m_leftMaster.set(ControlMode.PercentOutput, 0);
		m_rightMaster.set(ControlMode.PercentOutput, 0);
	}
	
	public void addDesiredVelocities(double leftVel, double rightVel) {
		m_leftDesiredVel = leftVel;
		m_rightDesiredVel = rightVel;
	}

	public void setPositionMotionMagic(double leftPosition, double rightPosition) {
		m_leftMaster.set(ControlMode.MotionMagic, leftPosition);
		m_rightMaster.set(ControlMode.MotionMagic, rightPosition);
	}
	
	public void setVelocity(double leftVel, double rightVel) {
		m_rightMaster.set(ControlMode.Velocity, rightVel);
		m_leftMaster.set(ControlMode.Velocity, leftVel);
	}
	
	public void resetEncoders() {
		m_leftMaster.setSelectedSensorPosition(0, 0, 0);
		m_rightMaster.setSelectedSensorPosition(0, 0, 0);
	}
	public double getLeftOutputVoltage() {
		return m_leftMaster.getMotorOutputVoltage();
	}
	
	public double getLeftMasterCurrent() {
		return m_leftMaster.getOutputCurrent();
	}
	
	public double getLeftMasterPosition() {
		return m_leftMaster.getSelectedSensorPosition(0);
	}
	
	public double getLeftMasterSpeed() {
		return m_leftMaster.getSelectedSensorVelocity(0);
	}
	
	
	public double getRightOutputVoltage() {
		return m_rightMaster.getMotorOutputVoltage();
	}
	
	public double getRightMasterCurrent() {
		return m_rightMaster.getOutputCurrent();
	}
	
	public double getRightMasterPosition() {
		return m_rightMaster.getSelectedSensorPosition(0);
	}
	
	public double getRightMasterSpeed() {
		return m_rightMaster.getSelectedSensorVelocity(0);
	}
	
	
	public double getRawYaw() {
		if (Robot.oi.getStartingDirection() == "Forwards") {
			return Pathfinder.boundHalfDegrees(m_nav.getYaw());
		}
		else {
			return Pathfinder.boundHalfDegrees(m_nav.getYaw());
		}
	}
	
	public void resetYaw() {
		m_nav.reset();
	}
	
	public double getAverageEncoderPosition() {
		return (getRightMasterPosition() + getLeftMasterPosition())/2;
	}
	
	public double getAngle() {
//		converts angle from -180 to 180 to 0 to 360	
//		sets positive y as 0 deg, robot's front is 0 deg
		return (360 - getRawYaw()) % 360;
		
	}
	
    public void initDefaultCommand() {
        setDefaultCommand(new ArcadeDrive());
    }   
    
	public void resetXY() {
		m_currentX = 0;
		m_currentY = 0;
	}
	
    public void calcXY() {
    	// calculate x,y coordinates when moving in straight lines and turning in place, DOES NOT WORK
    	double m_currentDistance = (m_rightMaster.getEncoderPositionFeet() + m_leftMaster.getEncoderPositionFeet())/2;
    	double m_distanceTraveled = (m_currentDistance - m_previousDistance);
    	double angle = getRawYaw();
    	m_currentX = m_currentX + m_distanceTraveled * Math.sin(Math.toRadians(angle));
    	m_currentY = m_currentY + m_distanceTraveled * Math.cos(Math.toRadians(angle));
    	m_previousDistance = m_currentDistance;
    }
    
    public double getXpos() {
    	return m_currentX;
    }
    
    public double getYpos() {
    	return m_currentY;
    }
	
	public double ticksToFeet(double ticks) {
		return m_leftMaster.ticksToFeet(ticks);
	}
	
	public double feetToTicks(double feet) {
		return m_leftMaster.feetToTicks(feet);
	}

	public double getLeftVelocityFeet() {
		return m_leftMaster.getLinearVelocity();
	}

	public double getRightVelocityFeet() {
		return m_rightMaster.getLinearVelocity();
	}

	public double getLeftPositionFeet() {
		return m_leftMaster.getEncoderPositionFeet();
	}

	public double getRightPositionFeet() {
		return m_rightMaster.getEncoderPositionFeet();
	}

	public void startVelocityController() {
		m_velocityNotifier.startPeriodic(DriveConstants.kVelocityPIDPeriod);
	}

	public void setTargetVelocities(double left, double right) {
		m_velocityPIDF.setVelocity(left, right);
	}

	public void stopVelocityPIDF() {
		m_velocityPIDF.stop();
		m_velocityNotifier.stop();
	}

    public void reportToSmartDashboard() {
    	SmartDashboard.putNumber("Left Master Voltage", getLeftOutputVoltage());
    	SmartDashboard.putNumber("Right Master Voltage", getRightOutputVoltage());
    	
    	SmartDashboard.putNumber("Left Master Position", getLeftMasterPosition());
    	SmartDashboard.putNumber("Right Master Position", getRightMasterPosition());
		
		SmartDashboard.putNumber("Left Master Position Feet", m_leftMaster.getEncoderPositionFeet());
		SmartDashboard.putNumber("Right Master Position Feet", m_rightMaster.getEncoderPositionFeet());		
		SmartDashboard.putNumber("Yaw", getRawYaw());
    	SmartDashboard.putNumber("X pos", m_currentX);
    	SmartDashboard.putNumber("Y pos", m_currentY);
    	
    }
    
    public void startLog() {
		writeException = false;
		// Check to see if flash drive is mounted.
		File logFolder1 = new File(m_filePath1);
		File logFolder2 = new File(m_filePath2);
		Path filePrefix = Paths.get("");
		if (logFolder1.exists() && logFolder1.isDirectory())
			filePrefix = Paths.get(logFolder1.toString(), m_fileName);
		else if (logFolder2.exists() && logFolder2.isDirectory())
			filePrefix = Paths.get(logFolder2.toString(),
					SmartDashboard.getString("log_file_name", m_fileName));
		else
			writeException = true;

		if (!writeException) {
			int counter = 0;
			while (counter <= 99) {
				m_file = new File(filePrefix.toString() + String.format("%02d", counter) + ".csv");
				if (m_file.exists()) {
					counter++;
				} else {
					break;
				}
				if (counter == 99) {
					System.out.println("file creation counter at 99!");
				}
			}
			try {
				m_writer = new FileWriter(m_file);
				m_writer.append("Time,RightPosition,LeftPosition,RightVelocity,LeftVelocity,RightDesiredVel,LeftDesiredVel,RightVoltage,LeftVoltage,"
						+ "RightMasterCurrent,LeftMasterCurrent,RightSlaveCurrent,LeftSlaveCurrent,BusVoltage,Yaw\n");
				m_writer.flush();
				m_logStartTime = Timer.getFPGATimestamp();
			} catch (IOException e) {
				e.printStackTrace();
				writeException = true;
			}
		}
	}

	public void stopLog() {
		try {
			if (null != m_writer)
				m_writer.close();
		} catch (IOException e) {
			e.printStackTrace();
			writeException = true;
		}
	}

	public void logToCSV() {
		if (!writeException) {
			try {
				double timestamp = Timer.getFPGATimestamp() - m_logStartTime;
				m_writer.append(String.valueOf(timestamp) + "," + String.valueOf(getRightMasterPosition()) + ","
						+ String.valueOf(getLeftMasterPosition()) + "," + String.valueOf(m_rightMaster.getLinearVelocity()) + ","
						+ String.valueOf(m_leftMaster.getLinearVelocity()) + "," + String.valueOf(m_rightDesiredVel) + "," + String.valueOf(m_leftDesiredVel)
						+ "," + String.valueOf(m_rightMaster.getMotorOutputVoltage())
						+ "," + String.valueOf(m_leftMaster.getMotorOutputVoltage()) + ","
						+ String.valueOf(m_rightMaster.getOutputCurrent()) + ","
						+ String.valueOf(m_leftMaster.getOutputCurrent()) + ","
						+ String.valueOf(m_rightSlave1.getOutputCurrent()) + ","
						+ String.valueOf(m_leftSlave1.getOutputCurrent()) + "," + String.valueOf(Robot.pdp.getVoltage()) + ","
						+ String.valueOf(getRawYaw()) + "\n");
				m_writer.flush();
			} catch (IOException e) {
				e.printStackTrace();
				writeException = true;
			}
		}
	}
}

