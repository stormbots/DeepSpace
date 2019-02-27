/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ChassisTeleopDrive extends Command {

  public double maxVelocity = 0;
  public double minAccelerationTime = Integer.MAX_VALUE;
  public double startTime = 0;
  public double currentTime = 0; 

  public ChassisTeleopDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.chassis);
    System.out.println("ChassisTeleopDrive Constructor ran");
  }

  // Called just before this Command runs the first time 
  @Override
  protected void initialize() {
    //set mode back to manual

    Robot.chassis.motorL1.follow(Robot.chassis.motorL0);
    System.out.println("ChassisTeleopDrive execute is running");
    maxVelocity = 0;
    minAccelerationTime = Integer.MAX_VALUE;
    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Robot.drive.driver.tankDrive(clamp(OI.getDriveFwdL(), -0.5, 0.5), clamp(OI.getDriveFwdR(), -0.5, 0.5)); //OI.getDriveFwdL()); // Use this one for TankDrive
    //Robot.drive.driver.tankDrive(OI.getDriveFwdL(), OI.getDriveFwdR()); //OI.getDriveFwdL()); // Use this one for TankDrive
    //Robot.drive.driver.arcadeDrive(OI.getDriveFwdL(), OI.getDriveSideR());// Caden's Arcade
    //Robot.drive.driver.arcadeDrive(clamp(OI.getDriveFwdR(), -0.5, 0.5), clamp(OI.getDriveSideL(), -0.5, 0.5));// Normal Arcade
    Robot.chassis.driver.arcadeDrive(Robot.m_oi.getDriveFwdR(), -Robot.m_oi.getDriveSideL(),true);// Normal Arcade
    //Robot.drive.driver.arcadeDrive(0.4, 0);
    //Robot.drive.motorL.set(0.3);
    //Robot.chassis.motorL0.set(0.5);
    //Robot.drive.motorL1.set(0.5);

    double currentVelocity = Math.abs(Robot.chassis.motorL0.getEncoder().getVelocity());
    currentTime = Timer.getFPGATimestamp();

    if(currentVelocity == 0) {
      startTime = Timer.getFPGATimestamp();
    }

    double accelerationTime = Integer.MAX_VALUE;

    if(maxVelocity == currentVelocity && maxVelocity > 5100) {
      accelerationTime = currentTime - startTime;
    }

    maxVelocity = Math.max(maxVelocity, currentVelocity);
    minAccelerationTime = Math.min(minAccelerationTime, accelerationTime);

    SmartDashboard.putNumber("Drive MaxAccelerationTime: ", minAccelerationTime);
    SmartDashboard.putNumber("Drive MaxAccelerationTime (Graph): ", minAccelerationTime);
    SmartDashboard.putNumber("Drive Velocity: ", maxVelocity);
    SmartDashboard.putNumber("Drive Velocity (Graph): ", maxVelocity);
    SmartDashboard.putNumber("Drive MaxAcceleration", maxVelocity/minAccelerationTime);
    // is about 5,300... exact data not gained yet.

    // [practice] exact(ish) Max Velocity:  5222.8574
    // [practice] exact(ish) Max Acceleration Time:  2.1198
    // [practice] exact(ish) Max Acceleration:  2463.8375
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
