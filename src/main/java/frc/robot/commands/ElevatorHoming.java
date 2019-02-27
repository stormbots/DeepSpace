/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ElevatorHoming extends Command {
  boolean elevHomed = false;
  
  public ElevatorHoming() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    setTimeout(4);
    Robot.armLift.arm.setAngle(-90);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(Robot.armLift.arm.isOnTarget(5)){
      Robot.armLift.elevator.elevMotor.set(ControlMode.PercentOutput, -0.2);
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    if(isTimedOut()){return true;}
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.armLift.elevator.elevMotor.set(ControlMode.PercentOutput, 0);
    //Robot.armLift.elevator.setMode(Mode.CLOSEDLOOP);
    elevHomed = true;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
