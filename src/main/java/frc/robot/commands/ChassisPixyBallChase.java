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
public class ChassisPixyBallChase extends Command {

  boolean areWeBroken = false;
  
  double imageBottom = 600;
  double imageCenterX = 400;

  double[] xValues={0,1,2};
  double[] yValues={0,1,2};
  double[] areas={0,1,2};
  double[] distances={0,1,2};

  int cyclesSinceLastLine = 100;

  double leftSidePower = 0;
  double rightSidePower = 0;

  public ChassisPixyBallChase() {
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

    leftSidePower = 0.3 - 0.4*Math.abs(startX);
    rightSidePower = 0.3 + 0.4*Math.abs(startX);
    leftSidePower *= 1.4; // 1.3..1.4 works
    rightSidePower *= 1.4;  // 1.3..1.4 works
  }

  public double getDistance(double x, double y) {
    return Math.sqrt((x-imageCenterX)*(x-imageCenterX) + (y-imageBottom)*(y-imageBottom));
  }

  public int getBestBallIndex() {

    int bestXIndex = 0;
    double bestX = xValues[0];
    // int bestDistanceIndex = 0;
    // double bestDistance = getDistance(xValues[0], yValues[0]);

    int secondBestXIndex = 0;
    double secondBestX = xValues[0];
    // int secondBestDistanceIndex = 0;
    // double secondBestDistance = getDistance(xValues[0], yValues[0]);

    for(int i = 1; i < xValues.length; i++) {
      if(Math.abs(xValues[i]) < Math.abs(bestX)) {
        secondBestXIndex = bestXIndex;
        secondBestX = bestX;
        bestXIndex = i;
        bestX = xValues[i];
      }
      // if(bestDistance < getDistance(xValues[i], yValues[i])) {
      //   secondBestDistanceIndex = bestDistanceIndex;
      //   secondBestDistance = bestDistance;
      //   bestDistanceIndex = i;
      //   bestDistance = getDistance(xValues[i], yValues[i]);
      // }
    }

    if(bestX + 20 > secondBestX) {
      if(getDistance(xValues[bestXIndex], yValues[bestXIndex]) > getDistance(xValues[secondBestXIndex], yValues[secondBestXIndex])) {
        return secondBestXIndex;
      }
    }
    return bestXIndex;
  }

  // save all our arrays
  // if none? 
  // find best (as function)
  // track


  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    //Line[] lines = SmartDashboard.getNumberArray("Grip/something, defaultValue);
    
    cyclesSinceLastLine++;

    // if(lines.length>0) {
    //   cyclesSinceLastLine = 0;
    //   line = lines[0];
    // }
    if(cyclesSinceLastLine > 5){
      //chassis speed  = 0
      //shortSidePower = 0.6*shortSidePower;
      //Robot.chassis.tankDrive(shortSidePower, shortSidePower);
      Robot.chassis.tankDrive(-0.3, -0.3);
      return ;
    }


    int bestBallIndex = getBestBallIndex();
    double ballX = xValues[bestBallIndex];
    double ballY = xValues[bestBallIndex];
    double ballArea = areas[bestBallIndex];

    double ballXNormalizedCenter = ballX - imageCenterX;

    leftSidePower = 0.3 + clamp(0.7*ballXNormalizedCenter, -0.3, 0.7);
    rightSidePower = 0.3 + clamp(0.7*ballXNormalizedCenter, -0.3, 0.7);

    Robot.chassis.tankDrive(leftSidePower, rightSidePower);

    // SmartDashboard.putNumber("Chassis/LeftPixyPower", leftPixyPower);
    // SmartDashboard.putNumber("Chassis/RightPixyPower", rightPixyPower);
    // SmartDashboard.putNumber("Chassis/biasToRight", leftPixyPower-rightPixyPower);

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
