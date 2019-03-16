/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commandgroups.DefenseModeDisengage;
import frc.robot.commandgroups.DefenseModeEngage;
import frc.robot.subsystems.ArmElevator.Mode;
import frc.robot.subsystems.ArmElevator.Pose;

public class DefenseModeSwitcher extends Command {

  CommandGroup engage = new DefenseModeEngage();
  CommandGroup disengage = new DefenseModeDisengage();

  boolean engaged = true; 
  public DefenseModeSwitcher() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //Try to figure out if we're locked inside the frame or not
    double wristAngle = Robot.armLift.wrist.getWristAngleFromFloor();
    double armAngle = Robot.armLift.arm.getArmAngle();
    if( wristAngle < -90 &&  armAngle< -80){
      engaged = true;
    }
    else{
      engaged = false;
    }

    // Poses could be used, but are only somewhat trustworthy because if 
    // annika hits a button like hatch1 and jams it, what happens!? D:
    // We're technically engaged in a recovery required position but the pose is HATCH_1
    // // engaged = Robot.armLift.pose == Pose.HIDE || Robot.armLift.pose == Pose.HIDE_2;

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(engaged){disengage.start();}
    else{engage.start();}
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !engage.isRunning() && !engage.isRunning();
  }

  // Called once after isFinished returns true
  @Override
  protected void end(){
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
