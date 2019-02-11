/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * Add your docs here.
 */
public class Hand extends Subsystem {

    public Solenoid leftHand = new Solenoid(2);
    public Solenoid rightHand = new Solenoid(3);

    public static final boolean OPEN = false; 
    public static final boolean CLOSE = !OPEN;

    public TalonSRX motor = new TalonSRX(14);
  
    // Put methods for controlling this subsystem
    // here. Call these from Commands.
  
    boolean deployed = false;
  
    public Hand(){
      deployed = false;
      /*leftHand.set(deployed);
      rightHand.set(deployed);
      */

      //We probably want a very low continuous current, to avoid causing harm to the cargo. 
      //this lets us keep it running without much concern, as we have no sensors
      motor.configPeakCurrentLimit(5, 10); // 35 A 
      motor.configPeakCurrentDuration(200, 10); // 200ms
      motor.configContinuousCurrentLimit(2, 10); // 30A
      motor.enableCurrentLimit(true); // turn it on
}

    public void open(){
      leftHand.set(OPEN);
      rightHand.set(OPEN);
    }

    public void close(){
      leftHand.set(CLOSE);
      rightHand.set(CLOSE);
    }

    public void intake(){
      motor.set(ControlMode.PercentOutput, 0.5);
    }
    public void eject(){
      motor.set(ControlMode.PercentOutput, -0.5);
    }

    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
      // setDefaultCommand(new MySpecialCommand());
    }
}