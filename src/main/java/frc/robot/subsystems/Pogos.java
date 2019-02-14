/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Pogos extends Subsystem {

  public Solenoid leftPogo = new Solenoid(0);
  public Solenoid rightPogo = new Solenoid(1);
  public DigitalInput onHabCenter = new DigitalInput(3);

  public VictorSPX pogoMotor = new VictorSPX(8);

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  boolean deployed = false;
  double motorPower;
  double velocity = 0.0;

  public Pogos(){

    deployed = false;
    leftPogo.set(deployed);
    rightPogo.set(deployed);
    
  }

  public void deployPogos(){
    deployed = true;
    leftPogo.set(deployed);
    rightPogo.set(deployed);
  }

  public void retractPogos(){
    deployed = false;
    leftPogo.set(deployed);
    rightPogo.set(deployed);
  }

  public void setPogoPower(double vel){
    motorPower = vel;
  }

  public void runPogoMotor(){
    pogoMotor.set(ControlMode.PercentOutput, motorPower);
  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
