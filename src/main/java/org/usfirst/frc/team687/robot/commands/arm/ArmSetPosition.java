package org.usfirst.frc.team687.robot.commands.arm;

import org.usfirst.frc.team687.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmSetPosition extends Command {

    private double m_desired;

    public ArmSetPosition(double desired) {
	requires(Robot.arm);
	m_desired = desired;
    }
    
    public void initialize() {
	SmartDashboard.putString("Current Command", "IntakeSetPostion");
    }

    @Override
    public void execute() {
    SmartDashboard.putNumber("desired position", m_desired);
	Robot.arm.setPosition(m_desired);
    }

    @Override
    protected boolean isFinished() {
    	return false;
    }

}
