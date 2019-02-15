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

  public Solenoid pogo = new Solenoid(2);
  public DigitalInput onHabCenter = new DigitalInput(3);

  public VictorSPX pogoMotor = new VictorSPX(15);

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public static final boolean DEPLOYED = false;
  public static final boolean RETRACTED = !DEPLOYED;
  double motorPower = 0.3;
  double velocity = 0.0;
  public static boolean help = false;

  public Pogos(){

    pogo.set(RETRACTED);
    help = true;
    System.out.println("Pogo is Initialized");
    
  }

  public void deployPogos(){
    
    pogo.set(DEPLOYED);
  }

  public void retractPogos(){
    pogo.set(RETRACTED);
  }

  public void setPogoPower(double pwr){
    motorPower = pwr;
  }

  public void update(){
    pogoMotor.set(ControlMode.PercentOutput, motorPower);
  }
    

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
