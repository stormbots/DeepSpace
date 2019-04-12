/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.stormbots.Lerp;
import com.stormbots.closedloop.FB;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis.Mode;

public class AutoDrive extends Command {

  double startTime = 0;
  double landingDelay;
  double mainMoveTime;
  double turnTime;

  Lerp tickToInch;

  Lerp startingAcceleration;
  
  Lerp leftTargetPos;
  Lerp rightTargetPos;

  Lerp leftTurnTarget;
  Lerp rightTurnTarget;
  


  public AutoDrive() {

    landingDelay = 1250;
    mainMoveTime = 1500;
    turnTime = 750;
    
    tickToInch = new Lerp(0, 40960, 0, 120);
    
    startingAcceleration = new Lerp(0, 500, 0, 1);
    leftTargetPos = new Lerp(landingDelay, landingDelay+mainMoveTime, 0, tickToInch.getReverse(155));
    rightTargetPos = new Lerp(landingDelay, landingDelay+mainMoveTime, 0, tickToInch.getReverse(170));

    // leftTurnTarget = new Lerp(0)

    // Use requires() here to declare subsystem dependencies
    requires(Robot.chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startTime = Timer.getFPGATimestamp();

    Robot.chassis.setMode(Mode.TANKDRIVE);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double runTime = Timer.getFPGATimestamp() - startTime;

    double powerL = 0;
    double powerR = 0;

    if(runTime < landingDelay) { // 1.25 sec at end
      powerL = Math.min(startingAcceleration.get(runTime), 1);
      powerR = Math.min(startingAcceleration.get(runTime), 1);

      Robot.chassis.getLeftEncoder().setPosition(0);
      Robot.chassis.getRightEncoder().setPosition(0);
    }
    else if(runTime < landingDelay+mainMoveTime) {
      powerL = FB.fb(leftTargetPos.get(runTime), Robot.chassis.getLeftEncoder().getPosition(), 0.02);
      powerR = FB.fb(rightTargetPos.get(runTime), Robot.chassis.getRightEncoder().getPosition(), 0.02);
    }

    Robot.chassis.tankDrive(powerL, powerR);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.chassis.setMode(Mode.ARCADEDRIVE);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.chassis.setMode(Mode.ARCADEDRIVE);

  }
}
