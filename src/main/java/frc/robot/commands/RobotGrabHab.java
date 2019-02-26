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

  public RobotGrabHab(double moveTime) {
    this.moveTime = moveTime;
    timeToAngle = new Lerp(0,moveTime, Robot.intake.PIVOT_REST, Robot.intake.PIVOT_MIN_HAB);
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
    requires(Robot.pogos);
    requires(Robot.chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    System.out.println("RobotGrabHab has initialized");
    Robot.pogos.deployPogos();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentTime = Timer.getFPGATimestamp() - startTime;
    Robot.intake.setTargetPosition(timeToAngle.get(currentTime));
    Robot.chassis.driver.tankDrive(0.2, 0.2);
    Robot.intake.setRollerPower(0.3);

    if(Robot.pogos.onHabCenter.get()) {
      Robot.pogos.retractPogos();
    }
    
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
    Robot.chassis.driver.tankDrive(0, 0);

    // NOTE:  NEED TO BE ABSOLUTELY SURE BEFORE ALLOWING THIS INTO THE CODE !!!!!!!!!!!!!!
    //Robot.pogos.retractPogos();

    System.out.println("RobotGrabHab has ended");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.chassis.driver.tankDrive(0, 0);
    System.out.println("RobotGrabHab has ended");

    // NOTE:  DO NOT allow the pogos to retract, or we will topple over the edge and destroy the robot
  }
}
