/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.stormbots.devices.pixy2.Line;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

/**
 * An example command.  You can replace me with your own command.
 */
public class ChassisPixyDrive extends Command {

  boolean areWeBroken = false;
  Line line = new Line();
  int cyclesSinceLastLine = 100;

  public ChassisPixyDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drive);
    System.out.println("PLEASE SEE ME!!!!");
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.pixy.setLamp(true, false);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    Line[] lines = Robot.pixy.getMainFeatures(0,1);

    cyclesSinceLastLine++;

    if(lines.length>0) {
      cyclesSinceLastLine = 0;
      line = lines[0];
    }
    if(cyclesSinceLastLine > 10){
      //chassis speed  = 0
      Robot.drive.driver.tankDrive(0, 0);
      return ; 
    }

    //Here we can trust a line regardless of what frame it was from

    System.out.println(line + "   " + cyclesSinceLastLine);
    line.normalizeBottom();
    double startX = line.x0;
    double startY = line.y0;
    double endX = line.x1;
    double endY = line.y1;
    double midX;
    double midY;

    double farPoint;
    double closePoint;
    double offset = 0;

    // is the line across the center? if so make it on the posotive side
    if(startX * endX < 0) {
      offset = Math.abs(Math.min(startX, endX));
      startX += offset;
      endX += offset;
    }
    
    // figure out what point to use
    if(Math.abs(startX) == Math.max(Math.abs(startX), Math.abs(endX))) {
      farPoint = startX;
      closePoint = endX;
    }
    else {
      farPoint = endX;
      closePoint = startX;
    }
    // use those points to find the midpoint
    midX = ((farPoint - closePoint) / 2) + closePoint;

    // reset the points and asjust the midpoint if it was changed at the beginning
    if(offset > 0) {
      midX -= offset;
      startX -= offset;
      endX -= offset;
    }

    // figure out which points to use
    if(Math.abs(startY) == Math.max(Math.abs(startY), Math.abs(endY))) {
      farPoint = startY;
      closePoint = endY;
    }
    else {
      farPoint = endY;
      closePoint = startY;
    }
    // use those points we figured out
    midY = ((farPoint - closePoint) / 2) + closePoint;


    if(midX > 0) {
      //Robot.drive.driver.tankDrive(0.2*midY + 0.5*Math.abs(midX), 0.2*midY); // IS GOOD
      Robot.drive.driver.tankDrive(-0.2*midY, -0.2*midY - 0.5*Math.abs(midX));
    }
    else {
      //Robot.drive.driver.tankDrive(0.2*midY, 0.2*midY + 0.5*Math.abs(midX)); // IS GOOD
      Robot.drive.driver.tankDrive(-0.2*midY - 0.5*Math.abs(midX), -0.2*midY);
    }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.pixy.setLamp(false, false);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.pixy.setLamp(false, false);
  }
}
