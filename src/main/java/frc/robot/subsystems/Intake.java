/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.stormbots.Clamp.*;

import com.stormbots.Lerp;
import com.stormbots.closedloop.FB;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDBase;
import edu.wpi.first.wpilibj.command.Subsystem;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  //Pivot 
  private CANSparkMax motorPivot = new CANSparkMax(7, MotorType.kBrushless);
  private CANEncoder encPivot = new CANEncoder(motorPivot);
  private DigitalInput limitPivot = new DigitalInput(8);
  //Roller
  private CANSparkMax motorRollers = new CANSparkMax(8, MotorType.kBrushless);
  //PassThrough
  private DigitalInput ballSensor = new DigitalInput(9);


  double powerPivot = 0;
  double targetPosition = 0;

  double PIVOT_MIN= 15;
  double PIVOT_MIN_HAB= 0;
  double PIVOT_MAX = 90;
  
  public enum Mode {CLOSEDLOOP,MANUAL,HABLIFT,DISABLE};
  private Mode mode = Mode.MANUAL;
  private Lerp pivotToDegrees = new Lerp(0, 40, 90, 0);

  public Intake() {
    //Enable maybe if required // motorPivot.setInverted(false);
  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  
  public void update(){
    //setup variables and defaults
    double targetPosition = this.targetPosition;
    double currentPosition = getPosition();  
    
    //Check Soft Limits
    if(mode == Mode.HABLIFT){
      targetPosition = clamp(targetPosition,PIVOT_MIN_HAB,PIVOT_MAX);
    }
    else{
      targetPosition = clamp(targetPosition,PIVOT_MIN,PIVOT_MAX);
    }


    //State Machine things!

    switch(mode){
      case MANUAL:
        //Do nothing, use power provided from setPower();
        break;
      case CLOSEDLOOP:
      
        //run feedback function
        FB.fb(targetPosition, currentPosition, 0.1);
        break;
      case HABLIFT:
      
        break;
      case DISABLE: 
        powerPivot = 0;
        break;
      }
   
    //Check physical limits of motion
    if(powerPivot > 0 && isPivotLimitPressed()){ powerPivot = 0;}
    if(powerPivot < 0  && currentPosition < PIVOT_MIN) { powerPivot = 0;}
      //normal mode
      //power lift mode

    
    //set output power
    motorPivot.set(powerPivot);
  }


  public void setPower(double power){
    this.powerPivot = power;
  }
  
  public void setMode(Mode newMode){
    this.mode = newMode;
  }

  public void setTargetPosition(int target ){
    this.targetPosition = target;
  }

  /** Returns Degrees */
  public double getPosition(){
    return pivotToDegrees.get(encPivot.getPosition());
  }

  public boolean isOnTarget(int tolerance){
    return bounded(getPosition(),targetPosition-tolerance,targetPosition+tolerance);
  }

  /** True if ball is in passthrough */
  public boolean hasBall() { 
    return ballSensor.get() == true;
  }

  public boolean isPivotLimitPressed() { 
    return limitPivot.get() == true;
  }


}



//rollers set on and set off
