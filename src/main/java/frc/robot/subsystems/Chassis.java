/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
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

  // /*
  //Motors must be burshless or the robot will explode
  public CANSparkMax motorL = new CANSparkMax(1, MotorType.kBrushless);
  public CANSparkMax slaveL1 = new CANSparkMax(2, MotorType.kBrushless);
  public CANSparkMax slaveL2 = new CANSparkMax(3, MotorType.kBrushless);
  public CANSparkMax motorR = new CANSparkMax(4, MotorType.kBrushless);
  public CANSparkMax slaveR1 = new CANSparkMax(5, MotorType.kBrushless);
  public CANSparkMax slaveR2 = new CANSparkMax(6, MotorType.kBrushless);
  // */

  // public WPI_TalonSRX motorL = new WPI_TalonSRX(0);
  // public TalonSRX slaveL1 = new TalonSRX(1);
  // public TalonSRX slaveL2 = new TalonSRX(2);
  // public WPI_TalonSRX motorR = new WPI_TalonSRX(3);
  // public TalonSRX slaveR1 = new TalonSRX(4);
  // public TalonSRX slaveR2 = new TalonSRX(5);



  //Motor 1 and 4 are the leaders of their sides. 1 for left and 4 for right.
  public DifferentialDrive driver = new DifferentialDrive(motorL, motorR);

  //Gearbox Shifters
  public Solenoid shifterL = new Solenoid(2);
  public Solenoid shifterR = new Solenoid(3);

  // Use an Enum to define pnuematic truth values, so that you get good named values 
  // backed by type checking everywhere.
  public enum Gear{
    HIGH(true),
    LOW(false);
    private boolean bool;
    Gear(boolean bool){this.bool = bool;}
    public boolean bool(){return bool;};
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ChassisTeleopDrive());
  }

  /**
   * initializes the ramprates and sets the slave motors
   */
  public Chassis(){
    /*
    motorL.setRampRate(5.0);
    slaveL1.setRampRate(5.0);
    slaveL2.setRampRate(5.0);
    motorR.setRampRate(5.0);
    slaveR1.setRampRate(5.0);
    slaveR2.setRampRate(5.0);
*/
    double voltageRampRate = 0.075;
    // motorL.configOpenloopRamp(voltageRampRate, 30);
    // slaveL1.configOpenloopRamp(voltageRampRate, 30);
    // slaveL2.configOpenloopRamp(voltageRampRate, 30);
    // motorR.configOpenloopRamp(voltageRampRate, 30);
    // slaveR1.configOpenloopRamp(voltageRampRate, 30);
    // slaveR2.configOpenloopRamp(voltageRampRate, 30);

    // For the CANSparkMax's, we would use the smartCurrentLimit()
    //TODO: Figure out good current limits, and if we should use full linear range or the low cap
    // See http://www.revrobotics.com/content/sw/max/sw-docs/java/com/revrobotics/CANSparkMax.html#setSmartCurrentLimit(int,int,int)
    // motorL.setSmartCurrentLimit​(int stallLimit, int freeLimit, int limitRPM)
    //NOTE: Max power draw for  for Neo Brushless at stall is 105Amps
    //NOTE: Max free-spin speed is 5700 RPM

    //In an attempt to budget power across all motors, this is a safe start point that should
    // minimize brownouts
    int stallLimit = 180/6;
    int freeLimit = 220/6;
    int limitRPM = 5700/3;
    motorL.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    slaveL1.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    slaveL2.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    motorR.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    slaveR1.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);
    slaveR2.setSmartCurrentLimit(stallLimit, freeLimit, limitRPM);

    slaveL1.follow(motorL);
    slaveL2.follow(motorL);
    slaveR1.follow(motorR);
    slaveR2.follow(motorR);

    shift(Gear.LOW);
  }

  public void shift(Gear gear){
    shifterL.set(gear.bool());
    shifterR.set(gear.bool());
  }
}
