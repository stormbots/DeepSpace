/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmElevator.Mode;
import frc.robot.subsystems.ArmElevator.Pose;

public class WristHoming extends Command {

  public WristHoming() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intake);
    requires(Robot.armLift.arm);
    requires(Robot.armLift.wrist);
    requires(Robot.armLift.elevator);
    requires(Robot.hand);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.armLift.wrist.setMode(Mode.MANUAL);
    setTimeout(4);
    Robot.intake.setTargetPosition(90);
    // Robot.armLift.arm.setAngle(75); //TODO Do we need this?
    Robot.hand.setPosition(frc.robot.subsystems.Hand.Position.CLOSE);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //wait for arm
    if(Robot.armLift.arm.isOnTarget(5)){
      Robot.armLift.wrist.setPower(-0.2);
    }
    else{
      Robot.armLift.wrist.setPower(-0.1);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //we run out of time and give up
    //we hit the switch
    if(isTimedOut()){return true;}
    if(Robot.armLift.wrist.isLimitPressed()){return true;}
    
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armLift.wrist.setHomedReversed();
    Robot.armLift.wrist.setPower(0);
    Robot.armLift.wrist.setMode(Mode.CLOSEDLOOP);
    Robot.armLift.arm.set(Pose.LOAD_CARGO_PREP);
    
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.armLift.wrist.setPower(0);
    if(!Robot.isCompbot)end();
  }
}
