/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.stormbots.Clamp;
import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis.Mode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pogos;

/**
 * An example command.  You can replace me with your own command.
 */
public class RobotGrabHab3 extends Command {
  // double angle = 0;
  double moveTime;
  Lerp timeToIntakeAngle;
  Lerp timeToPogoPosition;
  double startTime = 0;

  public RobotGrabHab3(double moveTime) {
    this.moveTime = moveTime;
    // timeToIntakeAngle = new Lerp(0, moveTime, 110, Intake.PIVOT_MIN_HAB);
    timeToIntakeAngle = new Lerp(0, moveTime, 110, Intake.PIVOT_MIN_HAB);
    timeToPogoPosition = new Lerp(0, moveTime, Pogos.RETRACTED, Pogos.DEPLOY_HAB_3);
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
    Robot.intake.setMode(Intake.Mode.HABLIFT);
    Robot.chassis.setMode(Mode.TANKDRIVE);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double currentTime = Timer.getFPGATimestamp() - startTime;
    currentTime = Clamp.clamp(currentTime, 0, moveTime);
    boolean robotIsUp = currentTime >= moveTime;


    if( robotIsUp && Robot.pogos.isFloorDetected()){
      Robot.pogos.setPosition(Pogos.RETRACTED);
      Robot.intake.setRollerPower(0);
      Robot.chassis.tankDrive(0,0);
    }
    else if(robotIsUp) {
      Robot.intake.setRollerPower(1); // make a constant when time permits
      Robot.chassis.tankDrive(-0.1,-0.1);
    }
    else{
      Robot.intake.setTargetPosition(timeToIntakeAngle.get(currentTime));
      Robot.pogos.setPosition(timeToPogoPosition.get(currentTime));
      Robot.chassis.tankDrive(0, 0);
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
    // NOTE:  NEED TO BE ABSOLUTELY SURE BEFORE ALLOWING THIS INTO THE CODE !!!!!!!!!!!!!!
    //Robot.pogos.retractPogos();

    Robot.chassis.tankDrive(0, 0);
    Robot.pogos.setPosition(Pogos.RETRACTED);
    Robot.chassis.setMode(Mode.DRIVER);
    Robot.intake.setMode(Intake.Mode.CLOSEDLOOP);
    System.out.println("RobotGrabHab has ended");
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.chassis.tankDrive(0, 0);
    Robot.pogos.setPosition(Pogos.RETRACTED);
    Robot.chassis.setMode(Mode.DRIVER);
    Robot.intake.setMode(Intake.Mode.CLOSEDLOOP);
    System.out.println("RobotGrabHab has been interrupted");
    
    // NOTE:  DO NOT allow the pogos to retract, or we will topple over the edge and destroy the robot
  }
}
