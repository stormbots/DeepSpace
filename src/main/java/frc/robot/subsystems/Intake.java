/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static com.stormbots.Clamp.bounded;
import static com.stormbots.Clamp.clamp;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.stormbots.Lerp;
import com.stormbots.closedloop.FB;
import com.stormbots.filter.SimpleMovingAverage;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.IntakeRestPosition;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Intake extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // Pivot
  private CANSparkMax pivotMotor = new CANSparkMax(7, MotorType.kBrushless);
  private CANEncoder pivotEnc = new CANEncoder(pivotMotor);
  // Roller
  private TalonSRX rollerMotor = new TalonSRX(8);
  // PassThrough

  private SimpleMovingAverage outputFilter = new SimpleMovingAverage(4);
  
  double pivotPower = 0;
  double rollerPower = 0;
  double pivotTargetPosition = 0;

  //TODO Create new named tab motor output position

  // Example program

  /** Create a fake Gyro sendable to get a 0-360 rotationwidget */
  FakeGyro gyro = new FakeGyro();

  @Override 
  public void periodic(){
    SmartDashboard.putData("Intake/Position",gyro.set(getAngle()));
    SmartDashboard.putNumber("Intake/Current Pos", getAngle());
    SmartDashboard.putString("Intake/Command", getCurrentCommandName());
    SmartDashboard.putNumber("Intake/Target Pos",this.pivotTargetPosition);
    SmartDashboard.putString("Intake/Mode", mode.toString());
  }
  
  public enum Mode {CLOSEDLOOP,MANUAL,HABLIFT,DISABLE};
  private Mode mode = Mode.CLOSEDLOOP;
  /** Configured with a 49:1*49:1*84:24 gear reduction over a full rotation */

  //hope for gearbox  private Lerp pivotToDegrees = new Lerp(0, 1.0*(49.0*7.0*7.0*84.0/24.0),  0, 360.0);
  private Lerp pivotToDegrees = new Lerp(
    0, 42,
    0, 90);

    double kPivotGain = 0;  /* SET VIA CONSTRUCTOR/INIT, DO NOT USE */
    double kPivotGainHab = 0;  /* SET VIA CONSTRUCTOR/INIT, DO NOT USE */
    double kPivotFF = 0; /* SET VIA CONSTRUCTOR/INIT, DO NOT USE */
  //intake on ground is 19.439
  public static final double PIVOT_MIN = 15;
  // public static final double PIVOT_MIN_HAB = 20.5+5;
  // public static final double PIVOT_MAX = 110.0; //practice bot
  public static       double PIVOT_MAX = 135.0;
  public static final double PIVOT_GRAB_HAB_2 = 20;
  public static final double PIVOT_GRAB_HAB_3 = 20;
  public static final double PIVOT_REST = PIVOT_MAX-1;
  public static final double PIVOT_GRAB_CARGO = 71.5-2; //was 71.5 -- old were 61.5, 76.5
  public static final double ROLLER_GRAB_CARGO = 1.0;
  // public static final double ROLLER_GRAB_POWER = 0;

  public Intake() {
    //Enable maybe if required // motorPivot.setInverted(false);
    /* We do not have a safe on-boot homing process, and rely on
     * power cycles being in our full up start position.
     * As a result, must set targetPosition to the current postiion
     * to avoid glitching and unexpected movement on code reboots.
     */
    pivotTargetPosition = getAngle();

    kPivotFF = 0.09;
    kPivotGain = 0.08; //see RobotInit note
    kPivotGainHab = 0.095+0.015+0.01;

    pivotMotor.setIdleMode(IdleMode.kBrake);

    //TODO: Increase current restrictions after limit and motor checks
    //pivotMotor.setSmartCurrentLimit(5, 10, 6700/3);
    pivotMotor.set(0);


    //Rollers
    rollerMotor.set(ControlMode.PercentOutput, 0);
  }

  /** Runs on robot boot after network/SmartDashboard becomes available */
  public void robotInit(){

    if(Robot.isCompbot){
      rollerMotor.setInverted(true);
    }
    else{
      rollerMotor.setInverted(false);
      PIVOT_MAX = 110.0;
      kPivotGainHab = 0.095+0.015;
    }
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new IntakeRestPosition());
  }

  
  public void update(){
    
    //setup variables and defaults
    double targetPosition = this.pivotTargetPosition;
    double currentPosition = getAngle();  

    //Check Soft Limits
    targetPosition = clamp(targetPosition,PIVOT_MIN,PIVOT_MAX);

    switch(mode){
      case MANUAL:
        //Do nothing, use power provided from setPower();
        break;
      case HABLIFT: 
        // No change from CLOSEDLOOP from positional and current limits
        pivotPower = FB.fb(targetPosition, currentPosition, kPivotGainHab);
        break;
      case CLOSEDLOOP:
      
        //run feedback function
        if(targetPosition >= 120){targetPosition += 5;}
        pivotPower = FB.fb(targetPosition, currentPosition, kPivotGain)
         + Math.cos(currentPosition*(Math.PI/180.0 ))*kPivotFF;
        break;
      case DISABLE: 
        pivotPower = 0;
        break;
    }


    // outputFilter.put(pivotPower);
    // pivotPower = outputFilter.get();

    //Position block should fix it unless we're oscillating wildly
    // if(pivotPower < 0  && currentPosition < PIVOT_MIN) { pivotPower = 0;}
    // if(pivotPower > 0  && currentPosition > PIVOT_MAX) { pivotPower = 0;}
    
    // SmartDashboard.putNumber("Intake/Active Current", pivotMotor.getOutputCurrent());
    SmartDashboard.putNumber("Intake/Current Position(final)",getAngle());
    SmartDashboard.putNumber("Intake/Output Power",pivotPower);
    // SmartDashboard.putNumber("Intake/Target after increment",targetPosition);



    //set output power
    //pivotPower = Clamp.clamp(pivotPower, -0.1, 0.1);
    pivotMotor.set( -pivotPower);
    rollerMotor.set(ControlMode.PercentOutput,rollerPower);
  }
  
  public void setMode(Mode newMode){
    this.mode = newMode;
  }

  public void setTargetHeight(double target) {
    double angleDegrees = getAngle() - 30;
    double angleRadians = angleDegrees * Math.PI/180.0;
    double outsideLength = 17 * Math.sin(angleRadians);
    double offsetHeight = 7.5; // in
    setTargetPosition(outsideLength + offsetHeight);
  }

  public void setTargetPosition(double target ){
    this.pivotTargetPosition = target;
  }

  /** Returns Degrees */
  public double getAngle() {
    return PIVOT_MAX-pivotToDegrees.get(pivotEnc.getPosition()); 
  }

  public boolean isOnTarget(double tolerance){
    return bounded(getAngle(),pivotTargetPosition-tolerance,pivotTargetPosition+tolerance);
  }

  public boolean isPivotLimitPressed() {
    //TODO: Don't have limit switches, so we have to use some soft method
    // May consider current check on the "up" position, if it's safe to do so.
    return !bounded(getAngle(), PIVOT_MIN+1, PIVOT_MAX-1);
  }

  public void setRollerPower(double newPower){
    rollerPower = newPower;
  }
  public void setIntakePower(double newPower){
    pivotPower = newPower;
  }

  public void teleopInit() {
    outputFilter.clear();
  }

  public void setHomed(){
    pivotMotor.getEncoder().setPosition(0);
  }

}
