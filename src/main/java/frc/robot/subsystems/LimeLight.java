/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class LimeLight extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void init(){
    NetworkTable.getTable("limelight").putNumber("pipeline", 1);
    // sets the limelight's camera mode to camera mode
    NetworkTable.getTable("limelight").putNumber("camMode", 1);

  }

  public void update(){
    // grabs the camtran value (array)
    NetworkTable.getTable("limelight").getNumberArray("camtran", new double[6]);
    double[] camtran = NetworkTable.getTable("limelight").getNumberArray("camtran", new double[6]);

    // array values from the cmatran: (x,y,z,pitch,yaw,roll)
    double x = camtran[0]; // (x)
    double y = camtran[1]; // (y)
    double z = camtran[2]; // (z)
    double pitch = camtran[3]; // (x,z) green
    double yaw = camtran[4]; // (x,y) red
    double roll = camtran[5]; // (y,z) blue 

    // calculates pitch (not really needed)
    double h = Math.sqrt((x*x) + (z*z));
    double radianAngle = Math.acos(z/h);
    double angle = (radianAngle*180)/Math.PI;

    // prints out pitch (calculated and from networktable)
    System.out.println(angle); 
    System.out.println(pitch);
  }
}
