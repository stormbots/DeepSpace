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
import frc.robot.subsystems.ArmElevator.Pose;
import frc.robot.subsystems.Chassis.Mode;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pogos;

/**
 * An example command. You can replace me with your own command.
 */
public class RobotGrabHab3_v2 extends Command {
  // double angle = 0;
  double moveTime;
  Lerp timeToIntakeAngle;
  Lerp timeToPogoPosition;
  double startTime = 0;

  public RobotGrabHab3_v2(double moveTime) {
    this.moveTime = moveTime;
    // timeToIntakeAngle = new Lerp(0, moveTime, 100, Intake.PIVOT_GRAB_HAB_3);
    timeToIntakeAngle = new Lerp(0, moveTime, 100, Intake.PIVOT_GRAB_HAB_3);
    timeToPogoPosition = new Lerp(0, moveTime, Pogos.RETRACTED, Pogos.DEPLOY_HAB_3);
    // Use requires() here to declare subsystem dependencies
    requires(Robot.intake);
    requires(Robot.pogos);
    requires(Robot.chassis);
    requires(Robot.armLift.arm);
    requires(Robot.armLift.wrist);
    requires(Robot.armLift.elevator);
    requires(Robot.armLift);

    
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();
    // System.out.println("RobotGrabHab has initialized");

    // TankDrive Adaptations
    //Robot.chassis.setMode(Mode.TANKDRIVE);
    //Robot.chassis.tankDrive(0, 0);

    Robot.intake.setMode(Intake.Mode.HABLIFT);
    Robot.armLift.setPose(Pose.HATCH_1);
    Robot.pogos.kPogoGain = Pogos.POGO_GAIN_HAB;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double runTime = Timer.getFPGATimestamp() - startTime;
    double moveCurrentTime = Clamp.clamp(runTime, 0, moveTime);
    boolean robotIsUp = moveCurrentTime >= moveTime;

    if(runTime<moveTime){
      //gain elevation
      Robot.intake.setTargetPosition(timeToIntakeAngle.get(moveCurrentTime));
      Robot.pogos.setPosition(timeToPogoPosition.get(moveCurrentTime));
      Robot.intake.setRollerPower(0.5); // Check this... may be necesary
      //Robot.chassis.tankDrive(0, 0);
      Robot.chassis.arcadeDrive(0.2, 0);
    }
    else if(runTime < moveTime + 2){
      // All the way up
      Robot.intake.setRollerPower(1); // make a constant when time permits
      //Robot.chassis.tankDrive(0.3*0.8, 0.3*0.8);
      Robot.chassis.arcadeDrive(0.3,0);  
    }
    else if(runTime < moveTime + 4){
      //on hab, move intake up, pogos move up
      Robot.intake.setTargetPosition(Intake.PIVOT_REST);
      //TODO: This should probably be here to reduce oscillation and system damage
      // Robot.intake.setMode(Intake.Mode.CLOSEDLOOP);
  
      Robot.intake.setRollerPower(0);
      Robot.pogos.setPosition(Pogos.RETRACTED);
      //Robot.chassis.tankDrive(0, 0);
      Robot.chassis.arcadeDrive(0, 0);
    }
    else if(runTime < moveTime + 4 + 2 + 2){
      //move forward
      //Robot.chassis.tankDrive(0.2*0.8, 0.2*0.8);
      Robot.chassis.arcadeDrive(0.2,0); 
    }
    else if(runTime < moveTime + 4 + 2 + 1 + 1 + 1){
      //stop chassis
      //Robot.chassis.tankDrive(0, 0);
      Robot.chassis.arcadeDrive(0, 0);
      //TODO: This should probably be here to reduce oscillation and system damage
      // Robot.pogos.kPogoGain = Pogos.POGO_GAIN_IDLE;

    }
    else{
      //finished

      //Robot.chassis.tankDrive(0, 0);
      Robot.chassis.arcadeDrive(0, 0);

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

    // TankDrive Adaptations
    //Robot.chassis.tankDrive(0, 0);
    //Robot.chassis.setMode(Mode.ARCADEDRIVE);

    Robot.chassis.arcadeDrive(0, 0);
    Robot.pogos.setPosition(Pogos.RETRACTED);
    Robot.intake.setMode(Intake.Mode.CLOSEDLOOP);
    // System.out.println("RobotGrabHab has ended");
    Robot.pogos.kPogoGain = Pogos.POGO_GAIN_IDLE;

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
    // System.out.println("RobotGrabHab has been interrupted");
    
    // NOTE:  DO NOT allow the pogos to retract, or we will topple over the edge and destroy the robot
  }
  
}
