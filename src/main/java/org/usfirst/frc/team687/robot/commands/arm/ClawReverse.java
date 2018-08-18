package org.usfirst.frc.team687.robot.commands.arm;

import org.usfirst.frc.team687.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 *
 */
public class ClawReverse extends Command{
    
    public ClawReverse() {
	requires(Robot.claw);
    }
    
    @Override
    public void execute() {
	Timer.delay(0.254);
	if (Robot.claw.getClawState() == Value.kForward) {
		Robot.claw.clawReverse();
	}
	Timer.delay(0.687);
    }
    @Override
    protected boolean isFinished() {
	// TODO Auto-generated method stub
	return true;
    }

}
