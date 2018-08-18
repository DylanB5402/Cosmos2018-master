package org.usfirst.frc.team687.robot.subsystems;

import org.usfirst.frc.team687.robot.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 *
 */
public class Claw extends Subsystem {

    private DoubleSolenoid m_claw;

    
    public Claw() {
	    m_claw = new DoubleSolenoid(RobotMap.kClawSolenoidID2, RobotMap.kClawSolenoidID1);
    }
    
    public void clawForward() {
	    m_claw.set(DoubleSolenoid.Value.kForward);
    }
    
    public void clawReverse() {
	m_claw.set(DoubleSolenoid.Value.kReverse);
    }
    
    public boolean isClawOpen() {
	return m_claw.get() == DoubleSolenoid.Value.kForward;
    }
    
    public Value getClawState() {
    	return m_claw.get();
    }
    
    @Override
    protected void initDefaultCommand() {	
    }
    
    public void reportSmartDashboard() {
//	SmartDashboard.putBoolean("Claw State", isClawOpen());
//	SmartDashboard.putBoolean("******Has Cube", m_switch.get());
//	SmartDashboard.putString("Has Cube Test", "TEST IS A TEST");
//	SmartDashboard.putNumber("TestNumber", 687);

    }
    
}
