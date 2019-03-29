/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static com.stormbots.closedloop.FB.fb;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.stormbots.Clamp;
import com.stormbots.Lerp;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class Pogos extends Subsystem {

  // public Solenoid leftPogo = new Solenoid(5);
  public TalonSRX pogo = new TalonSRX(15); // TODO: SET Pogo DEVICE ID

  public DigitalInput onHabCenter = new DigitalInput(RobotMap.PogoFloorSensor); // TODO: SET hab DigitalInput ID

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // public static final double maxVelocity = 0; //fix this
  // public static final double maxAcceleration = 0;
  public static final double RETRACTED = -1;
  public static double DEPLOY_HAB_2 = 6; 
  public static double DEPLOY_HAB_3 = 19; // is the inches from the ground 
  public double kPogoGainInches = 0.7;
  public double targetPos = 0;
  double outputPower = 0;
  
  public static double POGO_GAIN_HAB = 0.02;
  public static double POGO_GAIN_IDLE = POGO_GAIN_HAB/4;
  public double kPogoGain = POGO_GAIN_IDLE;

  public static Lerp toInches = new Lerp(0, -76859, -1, 19);


  public enum Mode {CLOSEDLOOP,MANUAL};
  private Mode mode = Mode.CLOSEDLOOP;

  public Pogos(){
    pogo.setSensorPhase(true); //default for talon + encoder combo

    //TODO Pogo initialization? 
    // Since boot encoder == 0 and ideal targetPos == 0, we don't need to really do much
  }

  /** Runs on robot boot after network/SmartDashboard becomes available */
  public void robotInit(){
    if(Robot.isCompbot){
    }
    else{
    }
  }

  public void periodic(){
    SmartDashboard.putNumber("Pogos/currentPosition", pogo.getSelectedSensorPosition());
    SmartDashboard.putNumber("Pogos/outputPower", outputPower);
    SmartDashboard.putBoolean("Pogos/isOverGround", Robot.pogos.isFloorDetected());
  }


  public void update(){
    //TODO: Pogos should, as a safety, retract pogos if nothing is using them
    //if(this.getCurrentCommand() == null) targetPos = 0; //might 
    //optionally, could create a PogoPosition DefaultCommand and set the targetPos in the init

    //TODO: we should clamp the targetPos to within the bounds of the system, just in case
    // Note, that since we're being lazy about this and using ticks, we need to be aware that 
    // DEPLOYED might be negative, and so we'd have to check for that as part of our clamp process
    switch(mode){
      case CLOSEDLOOP:
      outputPower = fb(targetPos, pogo.getSelectedSensorPosition(0), kPogoGain);
      break;
      case MANUAL:
      break;
    }
    //Not varigied... but math says that it should work
    // double outputPower = fb(targetPos, toInches.get(pogo.getSelectedSensorPosition(0)), kPogoGainInches);


    //if(!SmartDashboard.containsKey("Pogos/outputpower")){SmartDashboard.putNumber("Pogos/outputpower", 0);}
    //outputPower = SmartDashboard.getNumber("Pogos/outputpower", 0);
    //TODO: Remove pogo safety clamp
    
    //outputPower = Clamp.clamp(outputPower, -0.2, 0.2);
    pogo.set(ControlMode.PercentOutput, outputPower);
    //pogo.set(ControlMode.PercentOutput, 0.1);
  }

  public void setPosition(double position){ // pass in as Inches
    this.targetPos = toInches.getReverse(position); // turn into ticks
  }
  public double getPosition(){
    return toInches.get(pogo.getSelectedSensorPosition());
  }

  public boolean isFloorDetected(){
    //TODO: Validate switch default vs triggerd state
    return onHabCenter.get() == false;
  }
  
  public void setPower(double power){
    outputPower = power;
  }

  public void setMode(Mode mode){
    this.mode = mode ;
  }

  public boolean isOnTarget(double tolerance){
    return Clamp.bounded(getPosition(),targetPos-tolerance,targetPos+tolerance);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
