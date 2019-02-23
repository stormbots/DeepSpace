/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmElevator.Mode;

public class ArmHoming extends Command {
  DigitalInput armLimit = new DigitalInput(1); //Only on compbot

  boolean wristHomed = false;

  public ArmHoming() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.intake);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(4);
    Robot.intake.setTargetPosition(90);
    //set arm
    // Robot.armLift.arm.set(Pose.LOAD_CARGO_PREP);
    Robot.armLift.arm.setAngle(75);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //wait for arm
    if(Robot.armLift.arm.isOnTarget(5)){
      Robot.armLift.wrist.setPower(-0.2);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //we run out of time and give up
    //we hit the switch
    if(isTimedOut()){return true;}
    if(!armLimit.get()){return true;}
    
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //set wrist as homed
    Robot.armLift.wrist.setPower(0);
    wristHomed = true;
    Robot.armLift.wrist.setMode(Mode.CLOSEDLOOP); }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
