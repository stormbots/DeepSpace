/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static com.stormbots.closedloop.FB.fb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Pogos extends Subsystem {

  //public Solenoid leftPogo = new Solenoid(5);
  public TalonSRX pogo = new TalonSRX(16); // NEED TO CHECK THE ACTUAL DEVICE ID THIS IS WRONG!!!!!!!--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  public DigitalInput onHabCenter = new DigitalInput(9);

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // public static final double maxVelocity = 0; //fix this
  // public static final double maxAcceleration = 0;
  public static final double maxPosition = 4096; // NEEDS TO BE TUNED
  public double fbConstant = 0.002;
  public double targetPos = 0;

  public Pogos(){
    System.out.println("Pogo is Initialized");  
  }

  /** Runs on robot boot after network/SmartDashboard becomes available */
  public void robotInit(){
    pogo.setSelectedSensorPosition(0);
    retractPogos();
    if(Robot.isCompbot){
    }
    else{
    }
  }

  // pushes the pogos down
  public void deployPogos() {
    targetPos = maxPosition;
  }

  // pulls the pogos up
  public void retractPogos() {
    targetPos = 0;
  }

  // public void setPogoPower(double pwr){
  //   pogo.set(ControlMode.PercentOutput, pwr);
  // }

  public void update(){
    pogo.set(ControlMode.PercentOutput, fb(targetPos, pogo.getSelectedSensorPosition(0), fbConstant));
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
