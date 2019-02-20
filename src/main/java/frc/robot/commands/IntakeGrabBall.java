/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PassThrough;

/**
 * An example command.  You can replace me with your own command.
 */
public class IntakeGrabBall extends Command {
  public boolean hasBall = false;
  public double hasBallTime = 0;
  public IntakeGrabBall() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
    //requires(Robot.passThrough);//TODO ask dan about the passthrough vs intake grab ball
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    hasBall = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // if(hasBall)return; //TODO this should work eventually

    Robot.intake.setTargetPosition(Intake.PIVOT_GRAB_CARGO);
    Robot.intake.setRollerPower(Intake.ROLLER_GRAB_CARGO);
    //Robot.passThrough.setPower(PassThrough.GRAB_BALL_POWER);//TODO ask dan about intake vs passthrough
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //if(hasBall == false && Robot.passThrough.hasBall()){
      if(true){
      //hasBall = true; 
      //hasBallTime = Timer.getFPGATimestamp();
      Robot.intake.setTargetPosition(Intake.PIVOT_REST);
    }
    //if(hasBall && (Timer.getFPGATimestamp() >= hasBallTime + 1.0)){
    //  Robot.passThrough.setPower(0);
    //}
    // return Timer.getFPGATimestamp() >= hasBallTime + 1.0;
    //TODO: Can't exit properly because if you do annika can't 
    /// let go of the button
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
