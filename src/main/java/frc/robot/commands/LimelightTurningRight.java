/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static com.stormbots.Lerp.lerp;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import frc.robot.subsystems.LimeLight;
/**
 * An example command.  You can replace me with your own command.
 */
public class LimelightTurningRight extends Command {
  public LimelightTurningRight() {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.m_subsystem);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    // sets the pipeline to the rightbrow targetting 
    NetworkTable.getTable("limelight").putNumber("pipeline", 1);

    // sets the limelight's camera mode to vision mode
    NetworkTable.getTable("limelight").putNumber("camMode", 0);

    // array of (x,y,z,pitch,yaw,roll)
    double[] camtran;

    // distance from target
    double z;

    // average calculated in iterations of 4, hopefully more zccurate than z
    double zAverage;

    // horizontal offset from the target to the center of the limelight's vision
    double[] txArray;
    double tx; // same thing as above

    // lerped tx to gain a turn adjustment value for turning 
    double turnAdjust;

    // constant proportional value for z (distance)
    double pz;
    
    // power of drive controlled using distance and a constant proportional
    double distancePowerMod = 0;

    // proportional for the turning adjustment used to lower the angle to close to 0
    double pTurnAdjust;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double[] txArray = NetworkTable.getTable("limelight").getNumberArray("tx", new double[0]);
    double tx = txArray[0];
    double turnAdjust = lerp(tx, -27, 27, -0.25, .25);
    double pTurnAdjust = 0.4; //Needs tuned 

    NetworkTable.getTable("limelight").getNumberArray("camtran", new double[6]);
    double[] camtran = NetworkTable.getTable("limelight").getNumberArray("camtran", new double[6]);
    double z = camtran[2];

    // Average, calculated on a 4 iteration basis and used to make the jitter z value 
    // taken from camtran from 3D compute more accurate.
    
    double sum = 0.0;
    for(int i = 0; i < 4; i++) {
      double value = z;
      sum += value;
    }
    double zAverage = sum / 4;

    double distancePowerMod = zAverage; //needs Lerp 

    // makes sure that the power of the drive does not exceed its limit
    if (distancePowerMod > 1){
      distancePowerMod = 1;
    }

    // makes us drive, should turn and then move forward in the same function
    Robot.drive.driver.tankDrive((0.5 * distancePowerMod) + (turnAdjust * pTurnAdjust), (0.5 * distancePowerMod) - (turnAdjust * pTurnAdjust));
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // initializes camtran stuff for the z value
    NetworkTable.getTable("limelight").getNumberArray("camtran", new double[6]);
    double[] camtran = NetworkTable.getTable("limelight").getNumberArray("camtran", new double[6]);
    double z = camtran[2];

    // calculates average for telling us when to stop
    double sum = 0.0;
    for(int i = 0; i < 4; i++) {
      double value = z;
      sum += value;
    }
    double zAverage = sum / 4;

    // tells us to stop when we get close enough to the target
    if (zAverage <= 18 ) {
      return true;
    }
    else {
      return false;
    }
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    // sets the limelight's camera mode to camera mode
    NetworkTable.getTable("limelight").putNumber("camMode", 1);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
