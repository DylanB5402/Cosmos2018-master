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

/**
 * Add your docs here.
 */
public class IntakeWheel extends Subsystem {

  private final TalonSRX m_intakeWheel;

  public IntakeWheel() {
    m_intakeWheel = new TalonSRX(RobotMap.kBigIntakeWheelID);
    m_intakeWheel.setInverted(true);
      m_intakeWheel.configNominalOutputForward(1, 0);
      m_intakeWheel.configNominalOutputReverse(0, 0);
    m_intakeWheel.setNeutralMode(NeutralMode.Brake);
    }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void setVoltage( double voltage) {
    m_intakeWheel.set(ControlMode.PercentOutput, voltage/12. );
  }
}


