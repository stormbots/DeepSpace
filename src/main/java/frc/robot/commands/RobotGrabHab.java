/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class RobotGrabHab extends Command {
  //double angle = 0;
  double moveTime;
  Lerp timeToAngle;
  double startTime = 0;
  double timeBellyPassed = 0;

  public RobotGrabHab(double moveTime) {
    this.moveTime = moveTime;
    timeToAngle = new Lerp(0,moveTime, Robot.intake.PIVOT_REST, Robot.intake.PIVOT_MIN_HAB);
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
    //requires(Robot.pogos);
    //requires(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    //Robot.pogos.deployPogos();
    System.out.println("RobotGrabHab has initialized");
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentTime = Timer.getFPGATimestamp() - startTime;
    Robot.intake.setTargetPosition(timeToAngle.get(currentTime));
    /*
    
    if(Robot.belly.get()) {
      if(timeBellyPassed == 0) timeBellyPassed = currentTime;
      
      Robot.drive.driver.tankDrive(0.2, 0.2);
    }

    // NOTE: the '+2000' is if the timer is in Milliseconds... DOUBLE CHECK BEFORE RUNNING
    if((timeBellyPassed != 0) && (timeBellyPassed+2000 < currentTime)) {
      Robot.pogos.setPogoPower(0);
    }
    else {
      Robot.pogos.setPogoPower(0.4);
    }

    */
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //TODO Do we exit the hab grab?
    // return Timer.getFPGATimestamp() - startTime > moveTime;
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //Robot.pogos.setPogoPower(0);
    //Robot.drive.driver.tankDrive(0, 0);

    // NOTE:  NEED TO BE ABSOLUTELY SURE BEFORE ALLOWING THIS INTO THE CODE !!!!!!!!!!!!!!
    //Robot.pogos.retractPogos();

    System.out.println("RobotGrabHab has ended");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    //Robot.pogos.setPogoPower(0);
    //Robot.drive.driver.tankDrive(0, 0);
    System.out.println("RobotGrabHab has ended");

    // NOTE:  DO NOT allow the pogos to retract, or we will topple over the edge and destroy the robot
  }
}
