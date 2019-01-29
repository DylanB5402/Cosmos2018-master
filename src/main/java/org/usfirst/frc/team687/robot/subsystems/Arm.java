package org.usfirst.frc.team687.robot.subsystems;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team687.robot.Robot;
import org.usfirst.frc.team687.robot.RobotMap;
import org.usfirst.frc.team687.robot.commands.arm.OpenLoopArm;
import org.usfirst.frc.team687.robot.constants.ArmConstants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Arm extends Subsystem {

    private final TalonSRX m_arm;

    private String m_filePath1 = "/media/sda1/logs/";
    private String m_filePath2 = "/home/lvuser/logs/";
    private File m_file;
    public FileWriter m_writer;
    private boolean writeException = false;
    private double m_logStartTime = 0;

    public Arm() {
    m_arm = new TalonSRX(RobotMap.kArmID);
    m_arm.configFactoryDefault();
	m_arm.setSensorPhase(false);
    m_arm.setInverted(true);
	m_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
	m_arm.config_kP(0, ArmConstants.kArmP, 0);
	m_arm.config_kI(0, ArmConstants.kArmI, 0);
    m_arm.config_kD(0, ArmConstants.kArmD, 0);
    m_arm.config_kF(0, ArmConstants.kArmF, 0);
	m_arm.setNeutralMode(NeutralMode.Brake);

    m_arm.configMotionAcceleration(ArmConstants.kArmAcceleration, 0);
    m_arm.configMotionCruiseVelocity(ArmConstants.kArmCruiseVelocity, 0);
	
//	m_arm.configForwardSoftLimitThreshold(GearIntakeConstants.kGearIntakeScorePos, 0);
//	m_arm.configReverseSoftLimitThreshold(GearIntakeConstants.kGearIntakeDownPos, 0);
//	m_arm.configForwardSoftLimitEnable(true, 0);
//	m_arm.configReverseSoftLimitEnable(true, 0);
	}

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new OpenLoopArm());
    }

    public void setPosition(double position) {
        m_arm.set(ControlMode.MotionMagic, position); //, 
            // DemandType.ArbitraryFeedForward, ArmConstants.kArmGravityFF * Math.cos(getAngle()));
    }
    public void setAngle(double angle) {
        setPosition(angleToTicks(angle));
    }

    public void setPower(double voltage){
        m_arm.set(ControlMode.PercentOutput, voltage);
    }
    
    public double angleToTicks(double angle){
        return  (angle - ArmConstants.kArmAngleZeroOffset) * ArmConstants.kTicksToDegrees;

    }

    public double ticksToAngle(double ticks){
        return (ticks / ArmConstants.kTicksToDegrees) +  ArmConstants.kArmAngleZeroOffset;
    }


    // // real world units
    // public double getPosition() {
    // return m_arm.getSelectedSensorPosition(0) / 4096;
    // }
    //
    // public double getSpeed() {
    // return m_arm.getSelectedSensorVelocity(0) * (600 / 4096);
    // }

    public double getPosition() {
	    return m_arm.getSelectedSensorPosition(0);
    }

    public double getVelocity() {
	    return m_arm.getSelectedSensorVelocity(0);
    }

    public void resetEncoder() {
	    m_arm.setSelectedSensorPosition(0, 0, 0);
    }

    public double getVoltage() {
	    return m_arm.getMotorOutputVoltage();
    }

    public double getCurrent() {
	    return m_arm.getOutputCurrent();
    }
    public double getAngle() {
        return ticksToAngle(getPosition());

    }
    

    public void reportState() {
	    SmartDashboard.putNumber("Arm Position", getPosition());
	    SmartDashboard.putNumber("Arm Speed", getVelocity());
	    SmartDashboard.putNumber("Arm Voltage", getVoltage());
	    SmartDashboard.putNumber("Arm Current", getCurrent());
        SmartDashboard.putNumber("Arm Angle", getAngle());
    }

    public void startLog() {

        // Check to see if flash drive is mounted.
    
        File logFolder1 = new File(m_filePath1);
        File logFolder2 = new File(m_filePath2);
        Path filePrefix = Paths.get("");
        if (logFolder1.exists() && logFolder1.isDirectory()) {
            filePrefix = Paths.get(logFolder1.toString(),
                Robot.kDate + "Arm");
    
        } else if (logFolder2.exists() && logFolder2.isDirectory()) {
    
            filePrefix = Paths.get(logFolder2.toString(),
                Robot.kDate + "Arm");
    
        } else {
    
            writeException = true;
    
        }
    
    
    
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
            m_writer.append("Time,Position,Velocity,Angle,Voltage,Current\n");
            m_logStartTime = Timer.getFPGATimestamp();

            } 
            catch (IOException e) {
            e.printStackTrace();
            writeException = true;
    
            }
    
        }
    
        }
    
    
    
        public void stopLog() {
        try {
            if (null != m_writer)
            m_writer.close();

        } 
        catch (IOException e) {
            e.printStackTrace();
            writeException = true;
    
        }
    
        }
    
    
    
        public void logToCSV() {
    
        if (!writeException) {
            try {
            double timestamp = Timer.getFPGATimestamp() - m_logStartTime;
            m_writer.append(String.valueOf(timestamp) +  ","
                + String.valueOf(getPosition()) + "," + String.valueOf(getVelocity()) + ","
                + String.valueOf(getAngle()) + "," + String.valueOf(getVoltage()) + ","
                + String.valueOf(getCurrent())  + "\n");
            m_writer.flush();
    
            } 
            catch (IOException e) {
    
            e.printStackTrace();
            writeException = true;
    
            }
    
        }
    
        }

}