/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmElevator.Pose;

public class LoadCargo extends Command {
  public LoadCargo() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.armLift);
    requires(Robot.armLift.arm);
    requires(Robot.armLift.wrist);
    requires(Robot.hand);
    //requires(Robot.passthru);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      Robot.armLift.setPose(Pose.LOAD_CARGO_PREP);
      //if on target ()
      Robot.armLift.setPose(Pose.LOAD_CARGO);
      //if on target
      //passtrough.setPower(passthrough.LOAD_POWER)
      //wait one second
      //
      //have a ball?
      //closegrabby
      //set
      Robot.armLift.setPose(Pose.LOAD_CARGO_PREP);
      //isontarget

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
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
