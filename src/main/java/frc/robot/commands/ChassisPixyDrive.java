/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.stormbots.devices.pixy2.Line;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

import static com.stormbots.Lerp.lerp;
import static com.stormbots.Clamp.clamp;

/**
 * An example command.  You can replace me with your own command.
 */
public class ChassisPixyDrive extends Command {

  boolean areWeBroken = false;
  Line line = new Line();
  int cyclesSinceLastLine = 100;

  public ChassisPixyDrive() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.chassis);

    System.out.println("ChassisPixyDrive Constructor ran");
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
      //Robot.drive.driver.tankDrive(0.4, 0.4);
      return ; 
    }

    //Here we can trust a line regardless of what frame it was from

    System.out.println(line + "   " + cyclesSinceLastLine);
    line.normalizeBottom();
    double startX = line.x0;
    double startY = line.y0;
    double endX = line.x1;
    double endY = line.y1;

    //adjusts to the first quadrant
    double startXq1 = lerp(startX, -1, 1, 0, 1);
    double endXq1 = lerp(endX, -1, 1, 0, 1);

    double k = 10.0; // needs some fixing

    double newX = -(endXq1 - startXq1) * k + startXq1;
    double newY = -(endY - startY) * k + startY;

    newX = lerp(newX, 0, 1, -1, 1);

    newX = clamp(newX, -1, 1);
    newY = clamp(newY, 0, 1);

    /*
    double midX;
    double midY;

    // System.out.println("start (X,Y): (" + startX + " , " + startY + ")");
    // System.out.println("end (X,Y): (" + endX + " , " + endY + ")");

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
    */

    if((Robot.chassis.sonarL.getRangeInches() + Robot.chassis.sonarR.getRangeInches()) / 2 < 6) {
      //Robot.drive.driver.tankDrive(0.2, 0.2);
    }
    else if(Math.sqrt(Math.pow(startX-endX, 2) + Math.pow(startY-endY, 2)) < 0.1) {
      //Robot.drive.driver.tankDrive(0.2, 0.2);
    }
    else {

      // known... can be set to this instead.
      //double shortSidePower = 0.5*startY;
      //double longSidePower = 0.5*startY + 2.0*Math.abs(startX);

      double shortSidePower = 0.5*newY;
      double longSidePower = 0.5*newY + 2.0*Math.abs(newX);

      if(startX > 0) {
        //Robot.drive.driver.tankDrive(0.2*midY + 0.5*Math.abs(midX), 0.2*midY); // IS GOOD
        //Robot.drive.driver.tankDrive(shortSidePower, longSidePower);
      }
      else if(startX < 0) {
        //Robot.drive.driver.tankDrive(0.2*midY, 0.2*midY + 0.5*Math.abs(midX)); // IS GOOD
        //Robot.drive.driver.tankDrive(longSidePower, shortSidePower);
      }
      else {
        //Robot.drive.driver.tankDrive(shortSidePower, shortSidePower);
      }
    }

    /*
    else{
      if(midX > 0) {
        //Robot.drive.driver.tankDrive(0.2*midY + 0.5*Math.abs(midX), 0.2*midY); // IS GOOD
        Robot.drive.driver.tankDrive(0.5*midY, 0.5*midY + 2.0*Math.abs(midX));
      }
      else {
        //Robot.drive.driver.tankDrive(0.2*midY, 0.2*midY + 0.5*Math.abs(midX)); // IS GOOD
        Robot.drive.driver.tankDrive(0.5*midY + 2.0*Math.abs(midX), 0.5*midY);
      }
    }
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
    Robot.pixy.setLamp(false, false);

  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    Robot.pixy.setLamp(false, false);
  }
}
