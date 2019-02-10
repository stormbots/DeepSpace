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
  //Roller
  private CANSparkMax rollerMotor = new CANSparkMax(8, MotorType.kBrushless);
  //PassThrough

  //Belt passthru
  private VictorSPX beltMotor = new VictorSPX(6);
  private VictorSPX beltMotorB = new VictorSPX(6);

  double beltPower = 0;
  double pivotPower = 0;
  double rollerPower = 0;
  double pivotTargetPosition = 0;

  
  public enum Mode {CLOSEDLOOP,MANUAL,HABLIFT,DISABLE};
  private Mode mode = Mode.CLOSEDLOOP;
  /** Configured with a 49:1*49:1*84:24 gear reduction over a full rotation */
  private Lerp pivotToDegrees = new Lerp(
     0, 0,     1.0*(49.0*7.0*7.0*84.0/24.0), 360.0
     );

  double kPivotGain = 0;  /* SET VIA CONSTRUCTOR/INIT, DO NOT USE */
  double kPivotFF = 0; /* SET VIA CONSTRUCTOR/INIT, DO NOT USE */
  double PIVOT_MIN= 15;
  double PIVOT_MIN_HAB = -10;
  double PIVOT_MAX = 110.0;


  public Intake() {
    //Enable maybe if required // motorPivot.setInverted(false);
    /* We do not have a safe on-boot homing process, and rely on
     * power cycles being in our full up start position.
     * As a result, must set targetPosition to the current postiion
     * to avoid glitching and unexpected movement on code reboots.
     */
    pivotTargetPosition = getPosition();

    //TODO: We may need to tune kPivot values on the smartdashboard
    kPivotFF = 0.05; // Holds itself stable quite well due to gearing
    kPivotGain = 0.004;

    //TODO: Increase current restrictions after limit and motor checks
    pivotMotor.setSmartCurrentLimit(2,4);

    //Rollers
    //TODO: Figure if this is needed: rollerMotorMotor.setInverted(false);
    rollerMotor.set(ControlMode.PercentOutput, 0);

    //Configure Passthrough belt motors
    //TODO: Figure if this is needed: beltMotor.setInverted(false);
    beltMotorB.follow(beltMotor);
    beltMotorB.setInverted(InvertType.OpposeMaster);
    beltMotor.set(ControlMode.PercentOutput, 0);

  }

  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  
  public void update(){
    //setup variables and defaults
    double targetPosition = this.pivotTargetPosition;
    double currentPosition = getPosition();  

    //Check Soft Limits
    if(mode == Mode.HABLIFT){
      targetPosition = clamp(targetPosition,PIVOT_MIN_HAB,PIVOT_MAX);
    }
    else{
      targetPosition = clamp(targetPosition,PIVOT_MIN,PIVOT_MAX);
    }

    switch(mode){
      case MANUAL:
        //Do nothing, use power provided from setPower();
        break;
      case HABLIFT: 
        // No change from CLOSEDLOOP from positional and current limits
      case CLOSEDLOOP:
      
        //run feedback function
        pivotPower = FB.fb(targetPosition, currentPosition, kPivotGain)
         + Math.cos(currentPosition*(Math.PI/180.0 ))*kPivotFF;
        break;
      case DISABLE: 
        pivotPower = 0;
        break;
      }
   
    //TODO Do we need to check check physical limits of motion?
    //Position block should fix it unless we're oscillating wildly
    //if(pivotPower < 0  && currentPosition < PIVOT_MIN) { pivotPower = 0;}
    

    System.out.println("TARGET: " + targetPosition);
    System.out.println("DEG: " + currentPosition);    
    System.out.println("PWR: " + pivotPower);

    //set output power
    pivotMotor.set(-pivotPower);
    rollerMotor.set(rollerPower);
    beltMotor.set(beltPower);
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
    return pivotToDegrees.get(-pivotEnc.getPosition());
  }

  public boolean isOnTarget(double tolerance){
    return bounded(getPosition(),pivotTargetPosition-tolerance,pivotTargetPosition+tolerance);
  }

  /** True if ball is in passthrough */
  public boolean hasBall() { 
    return  !bounded(beltMotor.getOutputCurrent(), -3, 3);
  }

  public boolean isPivotLimitPressed() {
    //TODO: Don't have limit switches, so we have to use some soft method
    // May consider current check on the "up" position, if it's safe to do so.
    return !bounded(getPosition(), PIVOT_MIN+1, PIVOT_MAX-1);
  }

  public void setRollerPower(double newPower){
    rollerPower = newPower;
  }

  public void setBeltPower(double newPower){
    beltPower = newPower;
  }

}
