/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static com.stormbots.Lerp.lerp;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import frc.robot.Robot; 


/**
 * An example command.  You can replace me with your own command.
 */
public class LimelightTurningLeft extends Command {
  
  public LimelightTurningLeft() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_subsystem);


  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
        // sets the pipeline to the leftbrow targetting 
    NetworkTable.getTable("limelight").putNumber("pipeline", 2);

    // grabs tx (horizontal offset from the target to the middle of the limelight's vision)
    // lerps tx and multiplies it by a proportional to creete a turnAdjust (NEEDS TUNING)
    double[] txArray = NetworkTable.getTable("limelight").getNumberArray("tx", new double[0]);
    double tx = txArray[0];
    double p = .5;
    double turnAdjust = lerp(tx, -27, 27, -1, 1) * p;
    
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /*
    Robot.drive.driver.tankDrive(0.5 + turnAdjust, 0.5 - turnAdjust);
    */
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