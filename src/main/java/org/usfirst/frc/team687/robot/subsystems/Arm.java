package org.usfirst.frc.team687.robot.subsystems;

import org.usfirst.frc.team687.robot.RobotMap;
import org.usfirst.frc.team687.robot.constants.ArmConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class Arm extends Subsystem {

    private final TalonSRX m_arm;
    public Arm() {
	m_arm = new TalonSRX(RobotMap.kArmID);
	m_arm.setSensorPhase(false);
	m_arm.setInverted(false);
	m_arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
	m_arm.config_kP(0, ArmConstants.kArmP, 0);
	m_arm.config_kI(0, ArmConstants.kArmI, 0);
	m_arm.config_kD(0, ArmConstants.kArmD, 0);
	m_arm.setNeutralMode(NeutralMode.Brake);
	
//	m_arm.configForwardSoftLimitThreshold(GearIntakeConstants.kGearIntakeScorePos, 0);
//	m_arm.configReverseSoftLimitThreshold(GearIntakeConstants.kGearIntakeDownPos, 0);
//	m_arm.configForwardSoftLimitEnable(true, 0);
//	m_arm.configReverseSoftLimitEnable(true, 0);
	}

    @Override
    protected void initDefaultCommand() {
    }

    public void setPosition(double pos) {
//	if (pos >= GearIntakeConstants.kGearIntakeUpPos) {
//	    pos = GearIntakeConstants.kGearIntakeUpPos;
//	}
//	if (pos <= GearIntakeConstants.kGearIntakeDownPos) {
//	    pos = GearIntakeConstants.kGearIntakeDownPos;
//	}
//    SmartDashboard.putNumber("pos desired 2", pos);
	m_arm.set(ControlMode.Position, pos);
    }

    public void setPower(double voltage) {
	m_arm.set(ControlMode.PercentOutput, voltage);
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

    public double getSpeed() {
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

    public void reportState() {
	SmartDashboard.putNumber("Arm Position", getPosition());
	SmartDashboard.putNumber("Arm Speed", getSpeed());
	SmartDashboard.putNumber("Arm Voltage", getVoltage());
	SmartDashboard.putNumber("Arm Current", getCurrent());
    }

}