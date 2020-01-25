/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Hand;

public class HandPose extends Command {
  boolean toggleOpen = false;
  Hand.Position position;
  double rollerPower=0;

  Hand.Position exitPosition;
  double exitRollerPower = 0;
  boolean finished = true;
  
  /** Toggle hand position */
  public HandPose() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.hand);
    toggleOpen = true;
  }

  /** For setting the hand power only */
  public HandPose(double rollerPower) {
    requires(Robot.hand);
    this.rollerPower = rollerPower;
  }

  /** For setting both position and power */
  public HandPose(Hand.Position position, double rollerPower) {
    requires(Robot.hand);
    this.position = position;
    this.rollerPower = rollerPower;
  }

  /** Exits only when cancelled, but allows you to set the position on killing a command */
  public HandPose(Hand.Position position, double rollerPower,Hand.Position exitPosition, double exitRollerPower) {
    requires(Robot.hand);
    this.position = position;
    this.rollerPower = rollerPower;
    this.finished = false;
    this.exitPosition = exitPosition;
    this.exitRollerPower = exitRollerPower;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    if(toggleOpen){
      Robot.hand.togglePosition();
      // System.out.println("Toggled");
    }else{
      if(position!=null)Robot.hand.setPosition(position);
      Robot.hand.setRollers(rollerPower);
    }
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return finished;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    if(exitPosition != null){
      Robot.hand.setPosition(exitPosition);
      Robot.hand.setRollers(exitRollerPower);
    }
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    if(exitPosition != null){
      Robot.hand.setPosition(exitPosition);
      Robot.hand.setRollers(exitRollerPower);
    }
  }
}
