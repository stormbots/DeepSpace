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
import frc.robot.subsystems.Pogos;

/**
 * An example command.  You can replace me with your own command.
 */
public class RobotGrabHab extends Command {
  // double angle = 0;
  double moveTime;
  Lerp timeToAngle;
  double startTime = 0;
  double timeBellyPassed = 0;

  public RobotGrabHab(double moveTime) {
    this.moveTime = moveTime;
    timeToAngle = new Lerp(0,moveTime, 90, Robot.intake.PIVOT_MIN_HAB);
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
    requires(Robot.drive);
    requires(Robot.pogos);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    Robot.pogos.deployPogos();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentTime = Timer.getFPGATimestamp() - startTime;
    boolean robotIsUp = currentTime > moveTime;


    if(!robotIsUp) {
      Robot.intake.setTargetPosition(timeToAngle.get(currentTime));
    }
    else{

      Robot.intake.setTargetPosition(timeToAngle.get(moveTime));

      if(Robot.bellySensor.get() && robotIsUp) {
        timeBellyPassed = currentTime;
        Robot.pogos.retractPogos();
      }
      else{
        Robot.pogos.setPogoPower(0.4);

      }

      // if(timeBellyPassed != 0) {
      //   Robot.drive.driver.tankDrive(0.4, 0.4);
      // }
      // else{
      //   Robot.drive.driver.tankDrive(0, 0);
      // }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //TODO Do we exit the hab grab?
    // need to know how far to drive forward;

    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.driver.tankDrive(0, 0);
    Robot.pogos.setPogoPower(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.drive.driver.tankDrive(0, 0);
    Robot.pogos.setPogoPower(0);
  }
}
