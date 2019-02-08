/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeSetPosition extends Command {
  double position = 0;
  double tolerance = 0;
  
  public IntakeSetPosition(double position) {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.intake);
    this.position = position;
  }

  public IntakeSetPosition(double position, double tolerance) {
    // Use requires() here to declare subsystem dependencies
    // requires(Robot.intake);
    this.position = position;
    this.tolerance = tolerance;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.intake.setTargetPosition(position);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //TODO: Return true if we're at the position
    if(tolerance <= 0 ) return true;
    if(Robot.intake.isOnTarget(tolerance)) return true;
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
