/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static com.stormbots.Clamp.clamp;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.OI;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ChassisTeleopDrive extends Command {
  public ChassisTeleopDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.chassis);
    System.out.println("ChassisTeleopDrive Constructor ran");
  }

  // Called just before this Command runs the first time 
  @Override
  protected void initialize() {
    //set mode back to manual
    System.out.println("ChassisTeleopDrive execute is running");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.drive.driver.tankDrive(clamp(OI.getDriveFwdL(), -0.5, 0.5), clamp(OI.getDriveFwdR(), -0.5, 0.5)); //OI.getDriveFwdL()); // Use this one for TankDrive
    //Robot.drive.driver.tankDrive(OI.getDriveFwdL(), OI.getDriveFwdR()); //OI.getDriveFwdL()); // Use this one for TankDrive
    //Robot.drive.driver.arcadeDrive(OI.getDriveFwdL(), OI.getDriveSideR());// Caden's Arcade
    //Robot.drive.driver.arcadeDrive(clamp(OI.getDriveFwdR(), -0.5, 0.5), clamp(OI.getDriveSideL(), -0.5, 0.5));// Normal Arcade
    Robot.chassis.driver.arcadeDrive(OI.getDriveFwdR(), -OI.getDriveSideL(),true);// Normal Arcade
    //Robot.drive.driver.arcadeDrive(0.4, 0);
    //Robot.drive.motorL.set(0.3);
    
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
