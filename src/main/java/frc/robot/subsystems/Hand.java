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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.commands.HandPower;

/**
 * Add your docs here.
 */
public class Hand extends Subsystem {

    public Solenoid hand = new Solenoid(0);

    public static final boolean OPEN = true; 
    public static final boolean CLOSE = !OPEN;

    //public static ShuffleboardTab handTab = Shuffleboard.getTab("Hand Data");

    public TalonSRX motor = new TalonSRX(14);
  
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public double handPwr = 0.0;
  
    public static final double GRAB_POWER = .5;
    public static final double HOLD_POWER = .1;
    public static final double EJECT_POWER = -0.5;
    public static final double OFF = .0;
    
    boolean deployed = false;
  
    public Hand(){
      close();
      deployed = false;
      /*leftHand.set(deployed);
      rightHand.set(deployed);
      */
      handPwr = 0.0;

      //We probably want a very low continuous current, to avoid causing harm to the cargo. 
      //this lets us keep it running without much concern, as we have no sensors
      motor.configPeakCurrentLimit(5, 10); // 35 A 
      motor.configPeakCurrentDuration(200, 10); // 200ms
      motor.configContinuousCurrentLimit(3, 10); // 30A
      motor.enableCurrentLimit(true); // turn it on
}

    public void open(){
      hand.set(OPEN);
    }

    public void close(){
      hand.set(CLOSE);
    }

    public void intake(){
      //motor.set(ControlMode.PercentOutput, 0.5);
      handPwr = 0.5;
    }
    public void eject(){
      //motor.set(ControlMode.PercentOutput, -0.5);
      handPwr = -0.5;
    }

    public void update(){
      motor.set(ControlMode.PercentOutput, handPwr);
    }

    public void setPower(double power){
      this.handPwr = power;
    }

    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
    }
}