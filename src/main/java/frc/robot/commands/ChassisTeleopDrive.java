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
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis.Mode;

/**
 * An example command.  You can replace me with your own command.
 */
public class ChassisTeleopDrive extends Command {

  double maxVelocity = Integer.MIN_VALUE;
  double minAccelerationTime = Integer.MAX_VALUE;
  double startTime = Integer.MIN_VALUE;

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
    Robot.chassis.setMode(Mode.ARCADEDRIVE);

    startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    
    Robot.chassis.arcadeDrive(OI.getDriveForward(), -OI.getDriveTurn());// Caden's Arcade 

    if(OI.driveStick.getRawButton(8)) {
      Robot.chassis.setMode(Mode.FOCUSDRIVE);
    }
    else {
      Robot.chassis.setMode(Mode.ARCADEDRIVE);
    }

    
    //Robot.chassis.arcadeDrive(0.4, 0);
    //Robot.drive.motorL.set(0.3);
    

    // BELOW THIS POINT IS DANGEROUS

    double currentVelocity = Math.abs(Robot.chassis.motorL0.getEncoder().getVelocity());
    double currentTime = Timer.getFPGATimestamp();

    if(currentVelocity == 0) {
      startTime = Timer.getFPGATimestamp();
    }

    double accelerationTime = Integer.MAX_VALUE;

    if(maxVelocity == currentVelocity && maxVelocity > 5100) {
      accelerationTime = currentTime - startTime;
    }

    maxVelocity = Math.max(maxVelocity, currentVelocity);
    minAccelerationTime = Math.min(minAccelerationTime, accelerationTime);


    SmartDashboard.putNumber("Chassis/Drive MaxAccelerationTime: ", minAccelerationTime);
    SmartDashboard.putNumber("Chassis/Drive Velocity: ", maxVelocity);
    SmartDashboard.putNumber("Chassis/Drive MaxAcceleration", maxVelocity/minAccelerationTime);
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
