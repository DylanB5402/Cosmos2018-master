package org.usfirst.frc.team687.robot.commands.arm;

import org.usfirst.frc.team687.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ArmSetPosition extends Command {

   // private double m_volt;
    private double m_desiredPos;
    public ArmSetPosition(double desired) {
	   // m_volt = voltage;
        m_desiredPos = desired;
        requires(Robot.arm);
    }
    
    public void initialize() {
	    SmartDashboard.putString("Current Command", "IntakeSetPostion");
       }

    @Override
    public void execute() {
        // Robot.arm.setPower(m_volt);
        Robot.arm.setPosition(m_desiredPos);
        
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

}
