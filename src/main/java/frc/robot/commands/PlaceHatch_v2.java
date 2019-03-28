/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.ArmElevator.Pose;
import frc.robot.subsystems.Hand.Position;

public class PlaceHatch_v2 extends Command {
  double startTime;
  double currentTime;
  public PlaceHatch_v2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.hand);
    requires(Robot.armLift.wrist);
    requires(Robot.armLift.elevator);
    requires(Robot.chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    currentTime = Timer.getFPGATimestamp() - startTime;
    //close hand
    //small delay
    //if hatch1, hatch2
      //rotate wrist down
    //if hatch3, elevatordown // maybe wait and move wrist up?

    //HATCH 3
    if(Robot.armLift.getPose() == Pose.HATCH_3){
      if(currentTime < 0.3){
        Robot.hand.setPosition(Position.CLOSE);
      }
      else if(currentTime < 0.8){
        Robot.armLift.elevator.setPosition(Robot.armLift.getPose().eleHeight()-2);
      }
      else if(currentTime < 1){
        Robot.chassis.arcadeDrive(-0.2, 0);
      }
      else if(currentTime < 2){
        Robot.chassis.arcadeDrive(0, 0);
      }
    }
    //ANY OTHER POSE
    else{
      if(currentTime < 0.3){
        Robot.hand.setPosition(Position.CLOSE);
      }
      else if(currentTime < 0.8){
        
        if(Robot.armLift.getPose() == Pose.HATCH_2){
          Robot.armLift.wrist.setTargetAngleFromFloor(-45);
          Robot.armLift.elevator.setPosition(Robot.armLift.getPose().eleHeight()-2);
        }
        else{
        Robot.armLift.wrist.setTargetAngleFromFloor(-10);
        }
      
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return currentTime > 2;
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
