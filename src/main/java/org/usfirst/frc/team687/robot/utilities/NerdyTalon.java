package org.usfirst.frc.team687.robot.utilities;

import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
/**
 * 
 * @author dbarv
 *Basic wrapper for TalonSRXs to make it easier to set PIDFs
 */

public class NerdyTalon extends TalonSRX {

	private double m_ticksPerFoot, m_ticksPerDegree;

	public NerdyTalon(int talonID) {
		super(talonID);
	}

	public void configDefaultSettings() {
		configVoltageCompensation(12);
		super.setStatusFramePeriod(StatusFrame.Status_1_General, 20, 0);
		super.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20, 0);
		super.configPeakOutputForward(1, 0);
		super.configPeakOutputReverse(-1, 0);
		super.configClosedloopRamp(0, 0);
		super.configOpenloopRamp(0, 0);	
	}
	
	public void configPIDF(double p, double i, double d, double f, int slot) {
		super.config_kP(slot, p, 0);
		super.config_kI(slot, i, 0);
		super.config_kD(slot, d, 0);
		super.config_kF(slot, f, 0);
	}

	public void configVoltageCompensation(double voltage) {
		super.configVoltageCompSaturation(voltage, 0);
		super.enableVoltageCompensation(true);
	}
	
	public void configPeakCurrentLimit(int current) {
		super.configPeakCurrentLimit(current, 0);
		super.enableCurrentLimit(true);
	}
	
	public void configMotionMagic(int accel, int cruise_vel) {
		super.configMotionAcceleration(accel, 0);
		super.configMotionCruiseVelocity(cruise_vel, 0);
	}
	
	/** 
	 *@param ticksPerFoot
	convert Talon Native Units to feet
	 */
	 
	public void configLinearUnits(double ticksPerFoot) {
		m_ticksPerFoot = ticksPerFoot;
	}
	
	public void configAngularUnits(double ticksPerDegree) {
		m_ticksPerDegree = ticksPerDegree;
	}

	public double getLinearVelocity() {
		return (super.getSelectedSensorVelocity(0) / 0.1) / m_ticksPerFoot;
	}

	public double getAngularVelocityDegrees() {
		return (super.getSelectedSensorVelocity(0) / 0.1) / m_ticksPerDegree;
	}

	public double ticksToFeet(double ticks) {
		return ticks / m_ticksPerFoot;
	}
	
	public double feetToTicks(double feet) {
		return feet * m_ticksPerFoot;
	}

	public double getEncoderPositionFeet() {
		return super.getSelectedSensorPosition(0) / m_ticksPerFoot;
	}

	public double degreesToTicks(double degrees) {
		return degrees * m_ticksPerDegree;
	}

	public double ticksToDegrees(double ticks) {
		return ticks / m_ticksPerDegree;
	}

	public double getEncoderAngleDegrees() {
		return super.getSelectedSensorPosition(0) / m_ticksPerDegree;
	}

	public double getAngularVelocityRadians() {
		return getAngularVelocityDegrees() * (Math.PI / 180);
	}


}
