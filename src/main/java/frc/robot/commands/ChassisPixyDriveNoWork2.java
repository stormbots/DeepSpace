/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static com.stormbots.Clamp.clamp;

import com.stormbots.devices.pixy2.Line;
import com.stormbots.interp.SinCurve;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis.Mode;

/**
 * An example command.  You can replace me with your own command.
 */
public class ChassisPixyDriveNoWork2 extends Command {

  boolean areWeBroken = false;
  Line line = new Line();
  int cyclesSinceLastLine = 100;

  double leftPixyPower = 0;
  double rightPixyPower = 0;


  public ChassisPixyDriveNoWork2() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.chassis);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.pixy.setLamp(true, false);
    Robot.chassis.setMode(Mode.TANKDRIVE);
  }

  private void followRawCloseLine(double x1, double y1, double x2, double y2) {

    if(y2 < y1) { 
      double x1staged = x1;
      double y1staged = y1;
      x1 = x2;
      x2 = x1staged;
      y1 = y2;
      y2 = y1staged;
    }


    //Do stuff! 
    double BASEPOWER = 0.05;
    double NEARBIAS  = 0.2; 
    double FARBIAS =   0.0;
    rightPixyPower = BASEPOWER;
    rightPixyPower += SinCurve.scurve(x1, 0, x2, 0.5, NEARBIAS); 
    rightPixyPower += y2*SinCurve.ncurve(x2, 0, 1, 0, FARBIAS);

    leftPixyPower = BASEPOWER; 
    leftPixyPower += SinCurve.scurve(-x1, 0, x2, 0.5, NEARBIAS); 
    leftPixyPower += y2*SinCurve.ncurve(-x2, 0, 1, 0, FARBIAS);


  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double distance = (Robot.chassis.sonarL.getRangeInches() + Robot.chassis.sonarR.getRangeInches()) / 2.0;

    Line[] lines = Robot.pixy.getMainFeatures(0,1);

    cyclesSinceLastLine++;

    if(lines.length>0) {
      cyclesSinceLastLine = 0;
      line = lines[0];
      line.normalizeBottom();
      
      }
    if(cyclesSinceLastLine > 5){
      Robot.chassis.tankDrive(-0.3, -0.3);
      return ;
    }

    //Here we can trust a line regardless of what frame it was from
    System.out.println(line + "   " + cyclesSinceLastLine);


    //followAugmentedCloseLine(line.x0, line.y0, line.x1, line.y1);
    followRawCloseLine(line.x0, line.y0, line.x1, line.y1);

    leftPixyPower = -clamp(leftPixyPower, -1, 1);
    rightPixyPower = -clamp(rightPixyPower, -1, 1);

    Robot.chassis.tankDrive(leftPixyPower, rightPixyPower);

    // SmartDashboard.putNumber("Chassis/LeftPixyPower", leftPixyPower);
    // SmartDashboard.putNumber("Chassis/RightPixyPower", rightPixyPower);
    // SmartDashboard.putNumber("Chassis/biasToRight", leftPixyPower-rightPixyPower);
    // SmartDashboard.putNumber("Chassis/PixyStartX", line.x0);
    // SmartDashboard.putNumber("Chassis/PixyStartY", line.y0);
    // SmartDashboard.putNumber("Chassis/PixyEndX", line.x1);
    // SmartDashboard.putNumber("Chassis/PixyEndY", line.y1);

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
    Robot.pixy.setLamp(false, false);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.chassis.setMode(Mode.ARCADEDRIVE);
    Robot.pixy.setLamp(false, false);
  }
}
