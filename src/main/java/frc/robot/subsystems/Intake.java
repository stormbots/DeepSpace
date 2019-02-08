/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import static com.stormbots.Clamp.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.stormbots.Clamp;
import com.stormbots.Lerp;
import com.stormbots.closedloop.FB;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PIDBase;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * An example subsystem.  You can replace me with your own Subsystem.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  //Pivot 
  private CANSparkMax pivotMotor = new CANSparkMax(5, MotorType.kBrushless); //origionally port 7
  private CANEncoder pivotEnc = new CANEncoder(pivotMotor);
  private DigitalInput pivotLimit = new DigitalInput(8);
  //Roller
  private CANSparkMax rollerMotor = new CANSparkMax(8, MotorType.kBrushless);
  //PassThrough
  private DigitalInput ballSensor = new DigitalInput(9);

  private VictorSPX beltMotor = new VictorSPX(6);
  //Belt passthru

  double beltPower = 0;
  double pivotPower = 0;
  double rollerPower = 0;
  double pivotTargetPosition = 0;

  double PIVOT_MIN= 15;
  double PIVOT_MIN_HAB= 0;
  double PIVOT_MAX = 90;
  
  public enum Mode {CLOSEDLOOP,MANUAL,HABLIFT,DISABLE};
  private Mode mode = Mode.MANUAL;
  private Lerp pivotToDegrees = new Lerp(0, 0.5, -90, 90);

  public Intake() {
    //Enable maybe if required // motorPivot.setInverted(false);
    //TODO Reset encoder somehow?
  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  
  public void update(){
    double kPivotGain = SmartDashboard.getNumber("IntakePivotGain", 0.035);
    double kPivotFF = SmartDashboard.getNumber("IntakePivotFFGain", 0.2445);

    kPivotFF = 0.2445;
    kPivotGain = 0.035;
    pivotTargetPosition = 90;
    //setup variables and defaults
    double targetPosition = this.pivotTargetPosition;
    double currentPosition = getPosition();  

    //Check Soft Limits
    if(mode == Mode.HABLIFT){
      targetPosition = clamp(targetPosition,PIVOT_MIN_HAB,PIVOT_MAX);
    }
    else{
      //TODO Re-enable targetPosition = clamp(targetPosition,PIVOT_MIN,PIVOT_MAX);
    }


    //State Machine things!
    mode = Mode.CLOSEDLOOP;//TODO REMOVE ME WHEN WE FINISH THE SUBSYSTEM

    switch(mode){
      case MANUAL:
        //Do nothing, use power provided from setPower();
        break;
      case CLOSEDLOOP:
      
        //run feedback function
        //powerPivot = FB.fb(targetPosition, currentPosition, kPivotGain);
        pivotPower = FB.fb(targetPosition, currentPosition, kPivotGain)
         + Math.cos(currentPosition*(Math.PI/180.0 ))*kPivotFF;
        break;
      case HABLIFT:
      
        break;
      case DISABLE: 
        pivotPower = 0;
        break;
      }
   
    //Check physical limits of motion
    // if(powerPivot > 0 && isPivotLimitPressed() ) { powerPivot = 0;}
    if(pivotPower < 0  && currentPosition < PIVOT_MIN) { pivotPower = 0;}
      //normal mode
      //power lift mode

    

    pivotPower = Clamp.clamp(pivotPower, -0.3, 0.3);


    System.out.println("TARGET: " + targetPosition);
    System.out.println("DEG: " + currentPosition);    
    System.out.println("PWR: " + pivotPower);

    //set output power
    pivotMotor.set(-pivotPower);
    rollerMotor.set(rollerPower);
  }


  public void setPower(double power){
    this.pivotPower = power;
  }
  
  public void setMode(Mode newMode){
    this.mode = newMode;
  }

  public void setTargetPosition(double target ){
    this.pivotTargetPosition = target;
  }

  /** Returns Degrees */
  public double getPosition(){
    return pivotToDegrees.get(pivotEnc.getPosition());
  }

  public boolean isOnTarget(double tolerance){
    return bounded(getPosition(),pivotTargetPosition-tolerance,pivotTargetPosition+tolerance);
  }

  /** True if ball is in passthrough */
  public boolean hasBall() { 
    return ballSensor.get() == true;
  }

  public boolean isPivotLimitPressed() { 
    return pivotLimit.get() == true;
  }

  public void setRollerPower(double newPower){
    rollerPower = newPower;
  }

  public void setBeltPower(double newPower){
    beltPower = newPower;
  }

}



//rollers set on and set off
