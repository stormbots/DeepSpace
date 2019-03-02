/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class Hand extends Subsystem {

    public Solenoid hand = new Solenoid(0);
    public Solenoid handB = new Solenoid(7);

    public enum Position{
      OPEN(false,true),
      CLOSE(true,false);
      private boolean compbot,practicebot;
      Position(boolean compbot, boolean practicebot){
        this.compbot = compbot;
        this.practicebot = practicebot;
      }
      public boolean bool(){return Robot.isCompbot ? this.compbot : this.practicebot;};
    }
    Position position = Position.CLOSE;
    
    // public static final boolean OPEN = false; //practice bot true
    // public static final boolean CLOSE = !OPEN;

    //public static ShuffleboardTab handTab = Shuffleboard.getTab("Hand Data");

    public TalonSRX motor = new TalonSRX(14);
  
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public double rollerPower = 0.0;
  
    public static final double GRAB_POWER = .5;
    public static final double HOLD_POWER = .1;
    public static final double EJECT_POWER = -0.5;
    public static final double OFF = .0;
    
  
    public Hand(){
      //We probably want a very low continuous current, to avoid causing harm to the cargo. 
      //this lets us keep it running without much concern, as we have no sensors
      motor.configPeakCurrentLimit(5, 10); // 35 A 
      motor.configPeakCurrentDuration(200, 10); // 200ms
      motor.configContinuousCurrentLimit(3, 10); // 30A
      motor.enableCurrentLimit(true); // turn it on
    }

    /** Runs on robot boot after network/SmartDashboard becomes available */
    public void robotInit(){
      setPosition(Position.CLOSE);

      if(Robot.isCompbot){
        motor.setInverted(true);
      }
      else{
        motor.setInverted(false);
      }
    }


    public void setPosition(Position position){
      this.position = position;
      hand.set(position.bool());
      handB.set(!position.bool());
      System.out.println(position);
      System.out.println(position.bool());
    }

    public Position getPosition(){
      return this.position;
    }
    
    public void togglePosition(){
      if(position == Position.CLOSE){
        setPosition(Hand.Position.OPEN);
      }else{
        setPosition(Hand.Position.CLOSE);
      }
    }

    public void setRollers(double power){
      rollerPower = power;
    }

    public void update(){
      SmartDashboard.putString("Hand/Command",getCurrentCommandName());
      motor.set(ControlMode.PercentOutput, rollerPower);
    }

    @Override
    public void initDefaultCommand() {
      // Set the default command for a subsystem here.
    }
}