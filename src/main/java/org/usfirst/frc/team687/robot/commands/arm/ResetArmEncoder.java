package org.usfirst.frc.team687.robot.commands.arm;

import org.usfirst.frc.team687.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ResetArmEncoder extends Command{

    public ResetArmEncoder() {
	requires(Robot.arm);
    }

    @Override
    public void execute() {
	Robot.arm.resetEncoder();
    }

    @Override
    protected boolean isFinished() {
	return false;
    }

}
