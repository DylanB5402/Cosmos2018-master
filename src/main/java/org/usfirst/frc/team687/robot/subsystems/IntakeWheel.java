/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team687.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.usfirst.frc.team687.robot.RobotMap;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class IntakeWheel extends Subsystem {

  private final TalonSRX m_intakeWheel;

  public IntakeWheel(int id) {
    m_intakeWheel = new TalonSRX(id);
    m_intakeWheel.configFactoryDefault();
    m_intakeWheel.setInverted(true);
      // m_intakeWheel.configNominalOutputForward(0, 0);
      // m_intakeWheel.configNominalOutputReverse(0, 0);
    m_intakeWheel.setNeutralMode(NeutralMode.Brake);
    }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setVoltage(double voltage) {
    m_intakeWheel.set(ControlMode.PercentOutput, voltage/12.0);
  }

  public void setPercentOutput(double power) {
    m_intakeWheel.set(ControlMode.PercentOutput, power);
  }

  public void reportToSmartDashboard1() {
    SmartDashboard.putNumber("Little intake voltage", m_intakeWheel.getMotorOutputVoltage());
  }

  public void reportToSmartDashboard2() {
    SmartDashboard.putNumber("Big intake voltage", m_intakeWheel.getMotorOutputVoltage());
  }
}


