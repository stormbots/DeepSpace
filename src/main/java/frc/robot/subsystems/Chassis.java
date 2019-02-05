/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.ChassisTeleopDrive;

/**
 * Add your docs here.
 */
public class Chassis extends Subsystem {

  public CANSparkMax motorL = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax slaveL1 = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax slaveL2 = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax motorR = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax slaveR1 = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax slaveR2 = new CANSparkMax(6, MotorType.kBrushless);

  public DifferentialDrive driver = new DifferentialDrive(motorL, motorR);

  public Solenoid shifterL = new Solenoid(1);
  public Solenoid shifterR = new Solenoid(2);
  

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ChassisTeleopDrive());
  }

  /**
   * initializes the ramprates and sets the slave motors
   */
  public Chassis(){
    motorL.setRampRate(5.0);
    slaveL1.setRampRate(5.0);
    slaveL2.setRampRate(5.0);
    motorR.setRampRate(5.0);
    slaveR1.setRampRate(5.0);
    slaveR2.setRampRate(5.0);

    slaveL1.follow(motorL);
    slaveL2.follow(motorL);
    slaveR1.follow(motorR);
    slaveR2.follow(motorR);
  }
}
