package org.usfirst.frc.team687.robot.commands.arm;

import org.usfirst.frc.team687.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class ClawForward extends Command{
    
    public ClawForward() {
	requires(Robot.claw);
    }
    
    public void initialize() {
	SmartDashboard.putString("Current Command", "ClawForward");
    }
    
    @Override
    public void execute() {
	Robot.claw.clawForward();
	Timer.delay(0.254);
    }
    @Override
    protected boolean isFinished() {
	// TODO Auto-generated method stub
	return true;
    }

}
