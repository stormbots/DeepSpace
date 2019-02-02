/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {

  public CANSparkMax motor1 = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax motor2 = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax motor3 = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax motor4 = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax motor5 = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax motor6 = new CANSparkMax(6, MotorType.kBrushless);

  public DifferentialDrive driver = new DifferentialDrive(motor1, motor4);

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public Chassis(){

    motor1.setRampRate(5.0);
    motor2.setRampRate(5.0);
    motor3.setRampRate(5.0);
    motor4.setRampRate(5.0);
    motor5.setRampRate(5.0);
    motor6.setRampRate(5.0);

  }

  public void forwardHalf(){

    motor1.set(0.5);
    
  }

  public void stop(){

    motor1.set(0);

  }

 
}
