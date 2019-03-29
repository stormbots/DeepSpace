/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static com.stormbots.Clamp.clamp;
import static com.stormbots.Lerp.lerp;

import com.stormbots.devices.pixy2.Line;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.Chassis.Mode;

/**
 * An example command.  You can replace me with your own command.
 */
public class ChassisPixyDrive extends Command {

  boolean areWeBroken = false;
  Line line = new Line();
  int cyclesSinceLastLine = 100;

  double shortSidePower = 0;
  double longSidePower = 0;

  public ChassisPixyDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.chassis);

    System.out.println("ChassisPixyDrive Constructor ran");
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.pixy.setLamp(true, false);
    Robot.chassis.setMode(Mode.TANKDRIVE);
  }


  private void followAugmentedCloseLine(double x1, double y1, double x2, double y2) {
    double startX = x1;
    double startY = y1;
    double endX = x2;
    double endY = y2;

    double startXq1 = lerp(startX, -1, 1, 0, 1);
    double endXq1 = lerp(endX, -1, 1, 0, 1);

    double k = 10.0; // needs some fixing

    double newX = -(endXq1 - startXq1) * k + startXq1;
    double newY = -(endY - startY) * k + startY;

    newX = lerp(newX, 0, 1, -1, 1);

    newX = clamp(newX, -1, 1);
    newY = clamp(newY, 0, 1);

    shortSidePower = 0.3 - 0.3*Math.sqrt(Math.abs(startX));
    longSidePower = 0.3 + 0.7*Math.sqrt(Math.abs(startX));
  }


  private void followRawCloseLine(double x1, double y1, double x2, double y2) {
    double startX = x1;
    double startY = y1;
    double endX = x2;
    double endY = y2;

    if(y2 > y1) { 
      double startXstaged = startX;
      double startYstaged = startY;
      startX = endX;
      endX = startXstaged;
      startY = endY;
      endY = startY;
    }

    shortSidePower = 0.3 - 0.4*Math.abs(startX);
    longSidePower = 0.3 + 0.4*Math.abs(startX);
    shortSidePower *= 1.3;
    longSidePower *= 1.3;
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
    }
    if(cyclesSinceLastLine > 5){
      //chassis speed  = 0
      //shortSidePower = 0.6*shortSidePower;
      //Robot.chassis.tankDrive(shortSidePower, shortSidePower);
      Robot.chassis.tankDrive(-0.3, -0.3);
      return ;
    }

    //Here we can trust a line regardless of what frame it was from

    System.out.println(line + "   " + cyclesSinceLastLine);
    line.normalizeBottom();

    //followAugmentedCloseLine(line.x0, line.y0, line.x1, line.y1);
    followRawCloseLine(line.x0, line.y0, line.x1, line.y1);

    longSidePower = -clamp(longSidePower, -1, 1);
    shortSidePower = -clamp(shortSidePower, -1, 1);
    
    double leftPixyPower = 0;
    double rightPixyPower = 0;

    if(line.x0 < 0) {
      leftPixyPower = shortSidePower;
      rightPixyPower = longSidePower;
    }
    else if(line.x0 > 0) {
      leftPixyPower = longSidePower;
      rightPixyPower = shortSidePower;
    }
    else {
      leftPixyPower = shortSidePower;
      rightPixyPower = shortSidePower;
    }
    
    // if(distance <= 4) {
    //   leftPixyPower = 0;
    //   rightPixyPower = 0;  
    // } else
    // if(Math.sqrt(Math.pow(line.x1-line.x0, 2) + Math.pow(line.y1-line.y0, 2)) < 0.1) {
    //   leftPixyPower = -0.2;
    //   rightPixyPower = -0.2;  
    // }

    Robot.chassis.tankDrive(leftPixyPower, rightPixyPower);

    SmartDashboard.putNumber("Chassis/LeftPixyPower", leftPixyPower);
    SmartDashboard.putNumber("Chassis/RightPixyPower", rightPixyPower);
    SmartDashboard.putNumber("Chassis/biasToRight", leftPixyPower-rightPixyPower);

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
