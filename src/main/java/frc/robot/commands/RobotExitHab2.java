/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pogos;

public class RobotExitHab2 extends Command {
  public RobotExitHab2() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.pogos);
    requires(Robot.intake);
    requires(Robot.chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //set pogo mode to power/manual
    //set intake mode to power/manual
    Robot.intake.setMode(Intake.Mode.MANUAL);
    Robot.pogos.setMode(Pogos.Mode.MANUAL);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    // apply power
    Robot.intake.setIntakePower(-0.1);
    Robot.pogos.setPower(-0.3);
    Robot.chassis.arcadeDrive(-0.15, 0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //whenever anika hits a button and exits it
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.pogos.setPosition(Pogos.RETRACTED);
    Robot.intake.setMode(Intake.Mode.CLOSEDLOOP);
    Robot.pogos.setMode(Pogos.Mode.CLOSEDLOOP);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
