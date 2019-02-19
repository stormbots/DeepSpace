/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.command.Command;

import frc.robot.*;
import frc.robot.subsystems.ArmElevator.Pose;
import frc.robot.subsystems.ArmElevator.*;

public class ArmPose extends Command {
  Pose pose;
  public ArmPose(Pose pose) {
    this.pose = pose;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    // These line can't be instantiated by robot, doesn't like nested subsystems?
    requires(Robot.armLift);
    requires(Robot.armLift.elevator);
    requires(Robot.armLift.wrist);
    requires(Robot.armLift.arm);
    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Avoid catastrophic pose changes if we're in the robot
    if(Robot.armLift.pose==Pose.LOAD_CARGO) pose = Pose.LOAD_CARGO_PREP;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.armLift.setPose(pose); 
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.armLift.isOnTarget();
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
