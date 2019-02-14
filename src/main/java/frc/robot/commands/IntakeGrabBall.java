/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PassThrough;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeGrabBall extends Command {
  public IntakeGrabBall() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
    requires(Robot.passThrough);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if(Robot.intake.hasBall())return; //TODO Do we need this?

    Robot.intake.setTargetPosition(Intake.PIVOT_GRAB_CARGO);
    Robot.intake.setRollerPower(Intake.ROLLER_POWER);
    Robot.passThrough.setPower(PassThrough.GRAB_BALL_POWER);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.passThrough.hasBall();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
