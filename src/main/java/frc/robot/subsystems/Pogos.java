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
  public static double HAB_2 = 6; 
  public static double DEPLOY_HAB_3 = 19; // is the inches from the ground 
  public double kPogoGain = 0.02;
  public double kPogoGainInches = 0.7;
  public double targetPos = 0;

  public static Lerp toInches = new Lerp(0, -34763, -1, 19);

  public Pogos(){
    System.out.println("Pogo is Initialized");  
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

  public void update(){
    //TODO: Pogos should, as a safety, retract pogos if nothing is using them
    //if(this.getCurrentCommand() == null) targetPos = 0; //might 
    //optionally, could create a PogoPosition DefaultCommand and set the targetPos in the init

    //TODO: we should clamp the targetPos to within the bounds of the system, just in case
    // Note, that since we're being lazy about this and using ticks, we need to be aware that 
    // DEPLOYED might be negative, and so we'd have to check for that as part of our clamp process

    double outputPower = fb(targetPos, pogo.getSelectedSensorPosition(0), kPogoGain);
    //Not varigied... but math says that it should work
    // double outputPower = fb(targetPos, toInches.get(pogo.getSelectedSensorPosition(0)), kPogoGainInches);


    //if(!SmartDashboard.containsKey("Pogos/outputpower")){SmartDashboard.putNumber("Pogos/outputpower", 0);}
    //outputPower = SmartDashboard.getNumber("Pogos/outputpower", 0);
    //TODO: Remove pogo safety clamp
    // outputPower = clamp(outputPower,-0.2,0.2);
    SmartDashboard.putNumber("Pogos/Output Power", outputPower);
    
    outputPower = Clamp.clamp(outputPower, -0.2, 0.2);

    SmartDashboard.putNumber("Pogos/currentPosition", Robot.pogos.pogo.getSelectedSensorPosition());
    pogo.set(ControlMode.PercentOutput, outputPower);
  }

  public void setPosition(double position){ // pass in as Inches
    this.targetPos = toInches.getReverse(position); // turn into ticks
  }

  public boolean isFloorDetected(){
    //TODO: Validate switch default vs triggerd state
    return onHabCenter.get() == false;
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
